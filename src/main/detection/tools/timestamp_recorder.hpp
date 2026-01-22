#include <chrono>
#include <fstream>
#include <string>
#include <iostream>
#include <fmt/chrono.h>
#include <fmt/format.h>

namespace tools 
{

class TimestampRecorder
{
public:
    TimestampRecorder(std::string folder_path_) ;

    ~TimestampRecorder() ;

    void record_timestamp();

private:
    std::ofstream outfile_;
    int count_ = 0;
    
};
}


