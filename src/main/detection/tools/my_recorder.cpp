#include "my_recorder.hpp"

namespace tools 
{
// ThreadSafeMat::ThreadSafeMat(std::mutex* m) : cv::Mat()
// {
//     m_ = m;
//     m_->lock()
// }

// ThreadSafeMat::~ThreadSafeMat()
// {
//     m_
// }

MyRecorder::MyRecorder(double fps, std::string folder_path)
{
    start_time_ = std::chrono::steady_clock::now();
    frame_count_ = 1;
  
    auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    video_path_ = fmt::format("{}/{}.avi", folder_path, file_name);
    fps_ = fps;
    // 启动写入线程
    writer_thread_ = std::thread(&MyRecorder::writeVideo, this);
   
    newest_index_.store(2);
}

MyRecorder::~MyRecorder()
{
    stop_flag_ = true;
    cv_.notify_all(); // 通知写入线程退出
    writer_thread_.join();

    video_writer_.release();
}

cv::Mat& MyRecorder::get_mat_for_write()
{
    // std::cout << "mat " << (newest_index_.load() + 1)%3 << std::endl;
    // std::cout << "return mat"<< &mats_[(newest_index_.load() + 1)%3] << std::endl;
    return mats_[(newest_index_.load() + 1)%3];
}

void MyRecorder::end_write()
{
    newest_index_.store((newest_index_.load() + 1)%3);
    if (!init_) { init(mats_[newest_index_.load()]); }
    std::unique_lock lock(writing_mutex_);
    cv_.notify_all(); 
}

void MyRecorder::init(cv::Mat img)
{
    auto fourcc = cv::VideoWriter::fourcc('H', '2', '6', '4');
    video_writer_ = cv::VideoWriter(video_path_, fourcc, fps_, img.size());

    init_ = true;
}

void MyRecorder::writeVideo()
{
    while (!stop_flag_)
    {
        // std::cout << "check" << std::endl;
        std::unique_lock<std::mutex> lock(mtx_);
        
        cv_.wait(lock, [this] { 
            double need_recorded_frame = delta_time(std::chrono::steady_clock::now(), start_time_) / (1.0 / fps_);
        return need_recorded_frame > frame_count_ || stop_flag_; });

        // std::cout << "After wait" << std::endl;
        int need_recorded_frame = delta_time(std::chrono::steady_clock::now(), start_time_) / (1.0 / fps_);
        if (stop_flag_ && need_recorded_frame >= frame_count_ )
            break;

        while (need_recorded_frame > frame_count_) {
            std::cout << "during wait - need_recorded_frame: " << need_recorded_frame 
            << "fps = " << fps_
            << ", frame_count_: " << frame_count_
            << ", stop_flag_: " << stop_flag_ 
            << ", delta time = " << delta_time(std::chrono::steady_clock::now(), start_time_) << std::endl;
            // std::cout << "write index "<< newest_index_.load() << std::endl;
            // std::cout << "write mat "<< &mats_[newest_index_.load()] << std::endl;
            // cv::imshow("buffer", mats_[newest_index_.load()]);
            // std::cout << "image size " << mats_[newest_index_.load()].size() << std::endl;
            video_writer_.write(mats_[newest_index_.load()]);
            frame_count_++;
        }
    }
}
}