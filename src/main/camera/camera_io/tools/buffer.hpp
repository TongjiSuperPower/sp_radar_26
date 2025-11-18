#ifndef BUFFER
#define BUFFER




#include <vector>
#include <array>
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <chrono>

#include <opencv2/opencv.hpp>
// #include "recorder.hpp"

namespace tools 
{
class Triple {
public:
    Triple ();
};

class DoubleBuffer {
public:
    DoubleBuffer(int fps, std::string folder_path) 
    : video_writer_(folder_path + "/test.avi", cv::VideoWriter::fourcc('H', '2', '6', '4'), fps, cv::Size(3072,2048))
    {}

    void Write(cv::Mat& image) {
        std::lock_guard<std::mutex> write_lock(write_mutex_);
        const int write_idx = 1 - current_read_idx_.load(std::memory_order_relaxed);
        std::unique_lock<std::shared_mutex> buffer_lock(mutexes_[write_idx]);
        buffers_[write_idx] = image.clone();
        current_read_idx_.store(write_idx, std::memory_order_release);
    }

    void Read(cv::Mat& image) {
        const int read_idx = current_read_idx_.load(std::memory_order_acquire);
        std::shared_lock<std::shared_mutex> buffer_lock(mutexes_[read_idx]);
        // image = buffers_[read_idx].clone();
        auto time = std::chrono::steady_clock::now();
        if (!buffers_[read_idx].empty()) {
            // recorder_.record(buffers_[read_idx], time);
            video_writer_.write(buffers_[read_idx]);
        }
    }

private:
    std::array<cv::Mat, 2> buffers_;
    std::array<std::shared_mutex, 2> mutexes_;
    std::atomic<int> current_read_idx_{0};
    std::mutex write_mutex_;
    // tools::Recorder recorder_;
    cv::VideoWriter video_writer_;
};
}

#endif