#include "timer.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <stdexcept>

namespace tools
{
    // 使用高精度计时
    using Clock = std::chrono::high_resolution_clock;

    // 记录事件的开始时间
    void Timer::syn_start(const std::string &name)
    {
        if (events.find(name) == events.end())
        {
            events[name] = Event{name};
            event_names.push_back(name);
        }
        auto &event = events[name];
        event.count++;
        
        // 记录CPU时间
        event.cpu_start = std::chrono::high_resolution_clock::now();
    }

    // 记录事件的结束时间
    double Timer::syn_stop(const std::string &name)
    {
        auto &event = events[name];
        event.cpu_stop = std::chrono::high_resolution_clock::now();
        event.cpu_time += std::chrono::duration<double>(event.cpu_stop - event.cpu_start).count();
        return std::chrono::duration<double>(event.cpu_stop - event.cpu_start).count();
    }

    // // 异步事件开始（在GPU方面）
    // void Timer::asyn_start(const std::string &name)
    // {
    //     if (events.find(name) == events.end())
    //     {
    //         events[name] = Event{name};
    //         event_names.push_back(name);
    //     }
    //     auto &event = events[name];
    //     event.count++;
    //     // 创建CUDA事件
    //     cudaEventCreate(&event.gpu_start);
    //     cudaEventCreate(&event.gpu_stop);

    //     // 记录GPU事件开始时间
    //     cudaEventRecord(event.gpu_start, 0);
    // }

    // // 异步事件结束
    // double Timer::asyn_stop(const std::string &name)
    // {
    //     auto &event = events[name];

    //     // 记录GPU事件结束时间
    //     cudaEventRecord(event.gpu_stop, 0);
    //     cudaEventSynchronize(event.gpu_stop); // 等待异步操作完成

    //     // 计算GPU时间
    //     float time_ms = 0.0f;
    //     cudaEventElapsedTime(&time_ms, event.gpu_start, event.gpu_stop);
    //     event.gpu_time += time_ms / 1000.0; // 转换为秒

    //     // 销毁CUDA事件
    //     cudaEventDestroy(event.gpu_start);
    //     cudaEventDestroy(event.gpu_stop);
    //     return event.gpu_time;
    // }

    // 重置指定事件的计时器
    void Timer::reset(const std::string &name)
    {
        if (events.find(name) == events.end())
        {
            throw std::invalid_argument("Event not found: " + name);
        }
        auto &event = events[name];
        event.cpu_time = 0.0;
        event.gpu_time = 0.0;
        event.count = 0;
    }

    // 重置所有事件的计时器
    void Timer::reset()
    {
        for (auto &pair : events)
        {
            auto &event = pair.second;
            event.cpu_time = 0.0;
            event.gpu_time = 0.0;
            event.count = 0;
        }
    }

    // 打印所有事件的统计信息
    void Timer::print() const
    {
        std::cout << std::fixed << std::setprecision(4); // 输出精度设置

        for (const auto &name : event_names)
        {
            const auto &event = events.at(name);

            std::cout << "Event: " << name << "\n";
            if (event.cpu_time != 0)
            {
                std::cout << "  CPU Time: " << event.cpu_time << " seconds\n";
                std::cout << "  CPU average fps: " << static_cast<double>(event.count) / event.cpu_time << std::endl;
                std::cout<< "  CPU average time: " << event.cpu_time / event.count << std::endl;
            }
            if (event.gpu_time != 0)
            {
                std::cout << "  GPU Time: " << event.gpu_time << " seconds\n";
                std::cout << "  GPU average fps: " << static_cast<double>(event.count) / event.gpu_time << std::endl;
                std::cout<< "  GPU average time: " << event.gpu_time / event.count << std::endl;
            }
            std::cout << "  Count: " << event.count << "\n";
            std::cout << "-------------------------------------\n";
        }
    }

} // namespace tools
