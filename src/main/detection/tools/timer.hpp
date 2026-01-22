/*
计时器，根据事件名称，统计事件的耗时，提供cpu事件和gpu事件的统计
*/
#ifndef TOOLS_TIMER_HPP
#define TOOLS_TIMER_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
namespace tools
{
    class Timer
    {
    public:
        Timer() = default;
        ~Timer() = default;

        void syn_start(const std::string &name);
        double syn_stop(const std::string &name); // 返回一次事件的耗时
        void asyn_start(const std::string &name);
        double asyn_stop(const std::string &name); // 返回一次事件的耗时
        void reset(const std::string &name);
        void reset();
        void print() const;

    private:
        struct Event
        {
            std::string name;
            double cpu_time = 0.0;
            double gpu_time = 0.0;
            std::chrono::_V2::system_clock::time_point cpu_start, cpu_stop;
            cudaEvent_t gpu_start, gpu_stop;
            int count = 0;
        };

        std::unordered_map<std::string, Event> events;
        std::vector<std::string> event_names;
    };

} // namespace tools

#endif // TOOLS_TIMER_HPP