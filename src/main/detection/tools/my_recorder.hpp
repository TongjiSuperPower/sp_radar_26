#include <opencv2/opencv.hpp>
#include <thread>
#include <fmt/chrono.h>
#include <condition_variable>
#include "math_tools.hpp"

namespace tools 
{
// class ThreadSafeMat : public cv::Mat {
// public:
//     ThreadSafeMat(std::mutex *m);
//     ~ThreadSafeMat();
// private:
//     std::mutex *m_;
// };

class MyRecorder {
public:
    MyRecorder(double fps, std::string folder_path);
    ~MyRecorder();
    cv::Mat& get_mat_for_write();
    void end_write();
private:
    void init(cv::Mat);
    void writeVideo();


    std::array<cv::Mat, 3> mats_;
    // std::array<std::mutex, 3> muteces_;
    std::atomic<int> newest_index_;

    std::mutex writing_mutex_;

    std::chrono::steady_clock::time_point start_time_;
    int init_flag_;

    std::string video_path_;
    cv::VideoWriter video_writer_;  // 视频写入器
    std::thread writer_thread_;  // 写入视频的线程
    
    std::mutex mtx_;
    std::condition_variable cv_;
    int stop_flag_;

    int frame_count_, fps_, init_;
};
}