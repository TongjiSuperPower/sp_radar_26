#include "recorder.hpp"
#include <fmt/chrono.h>
#include <filesystem>
#include <string>
#include "math_tools.hpp"
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>

namespace tools
{
  Recorder::Recorder(double fps,std::string folder_path) : init_(false), fps_(fps) , folder_path_(folder_path)
  {
    start_time_ = std::chrono::steady_clock::now();
    last_time_ = start_time_;

  
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    video_path_ = fmt::format("{}/{}.avi", folder_path_, file_name);

    // 启动写入线程
    writer_thread_ = std::thread(&Recorder::writeVideo, this);
  }

  Recorder::~Recorder()
  {
    if (!init_)
      return;

    // 停止写入线程
    stop_flag_ = true;
    cv_.notify_all(); // 通知写入线程退出
    writer_thread_.join();

    video_writer_.release();
  }

  void Recorder::record(
      const cv::Mat &img,
      const std::chrono::steady_clock::time_point &timestamp)
  {
    if (!init_) init(img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  
  // 将图像帧和时间戳放入队列
  {
    std::lock_guard<std::mutex> lock(mtx_);
    frame_queue_.push({img, timestamp});
  }


  // 唤醒写入线程
  cv_.notify_all();
  }

  void Recorder::init(const cv::Mat &img)
  {
    auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());

    init_ = true;
  }

  void Recorder::writeVideo()
  {
    while (!stop_flag_)
    {
      std::unique_lock<std::mutex> lock(mtx_);
      cv_.wait(lock, [this]
               { return !frame_queue_.empty() || stop_flag_; });

      if (stop_flag_ && frame_queue_.empty())
        break;

      auto frame_data = frame_queue_.front();
      frame_queue_.pop();

      // 将帧写入视频
      video_writer_.write(frame_data.first);
    }
  }

} // namespace tools
