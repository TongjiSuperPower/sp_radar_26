#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <fstream>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <string>

namespace tools
{
  class Recorder
  {
  public:
    // 构造函数，初始化帧率
    explicit Recorder(double fps,std::string folder_path);

    // 析构函数，释放资源
    ~Recorder();

    // 记录图像和旋转四元数数据，并将图像帧写入视频
    void record(const cv::Mat &img, const std::chrono::steady_clock::time_point &timestamp);

  private:
    // 初始化VideoWriter和文本写入器
    void init(const cv::Mat &img);

    // 写入视频文件的线程函数
    void writeVideo();

    // 成员变量
    bool init_;  // 标记是否已经初始化
    double fps_;  // 帧率
    std::string folder_path_;  // 输出视频文件夹路径
    std::chrono::steady_clock::time_point start_time_;  // 开始时间
    std::chrono::steady_clock::time_point last_time_;  // 上一帧的时间

    std::string video_path_;  // 输出视频文件路径

    cv::VideoWriter video_writer_;  // 视频写入器

    std::atomic<bool> stop_flag_{false};  // 用于控制写入线程是否停止
    std::thread writer_thread_;  // 写入视频的线程

    // 用于线程间同步
    std::mutex mtx_;
    std::condition_variable cv_;
    std::queue<std::pair<cv::Mat, std::chrono::steady_clock::time_point>> frame_queue_;  // 存储待写入的视频帧
  };
}

#endif // RECORDER_HPP
