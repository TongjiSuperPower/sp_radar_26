#include "timestamp_recorder.hpp"

namespace tools
{
TimestampRecorder::TimestampRecorder(std::string folder_path_)
{
    std::string file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
    std::string full_path = fmt::format("{}/{}.txt", folder_path_, file_name);
    outfile_ = std::ofstream(full_path);
    // 打开文件准备写入
    if (!outfile_.is_open()) {
        std::cerr << "无法打开文件进行写入!" << std::endl;
    }
}

TimestampRecorder::~TimestampRecorder()
{
    if (outfile_.is_open()) {
        outfile_.close();
    }
}

void TimestampRecorder::record_timestamp()
{
    auto since_epoch = std::chrono::system_clock::now().time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(since_epoch).count();
    // std::cout << millis << std::endl;
    auto time_stamp_str = std::to_string(millis);
    outfile_ << count_ << ": " << time_stamp_str << std::endl;
    count_++;
}

} // namespace tools