#ifndef __TRACKER_HPP__
#define __TRACKER_HPP__

#include <array>
#include <map>
#include <list>
#include <vector>
#include <cstdint>
#include <chrono>
#include <fstream>
#include <string>
#include <rclcpp/time.hpp>
#include <rclcpp/clock.hpp>
#include <yaml-cpp/yaml.h>

#include "radar_msgs/msg/car.h"
#include "radar_msgs/msg/cars.hpp"

#include "../tools/extended_kalman_filter.hpp"
#include "hungarian_optimizer.h"

#define STATE_SIZE 4
#define MEASUREMENT_SIZE 2
#define ID_KINDS 12

const int HISTORY_SIZE = 10;
const double TIME_THRESHOLD = 1.5;

class Tracker : public tools::ExtendedKalmanFilter {
public:
    Tracker();
    void predict(rclcpp::Time time);
    void update(radar_msgs::msg::Car car, rclcpp::Time stamp);
    std::map<int, double> get_id_and_confidence();
    // 主导身份：对 history_ 做多数表决（-1 也是合法候选），没有多数返回 -1。
    // 关联判据和上报判据都用它，避免"关联时认领 X 但上报时又不报 X"。
    int dominant_id() const;
    // 可以对外上报的 id 及其置信度；没有多数则 {-1, 0}。一条轨迹最多上报一个 id。
    std::pair<int, double> best_id_and_confidence() const;
    // 窗口里这个 id 出现过几次。比多数表决松，用于"现任者还认不认得出这个 id"。
    int id_count_in_history(int id) const;
    // 历史里出现过两个以上合法 id = 中途抢过别人的观测。位掩码统计，可每帧无条件调用。
    bool has_multiple_ids() const;
    std::pair<double, double> get_position();
    
    double distance(radar_msgs::msg::Car car);
    bool is_near(radar_msgs::msg::Car car);
    // 匈牙利用的关联代价：欧氏距离 + 对"未成熟轨迹"的惩罚
    double association_cost(radar_msgs::msg::Car car);
    bool has_lost_track(rclcpp::Time now);

    // ---- 以下仅供诊断日志使用 ----
    uint64_t track_id() const { return track_id_; }
    int hits() const { return hits_; }
    // 当前门限半径：distance_threshold_ + v * dt_
    double gate_radius() const;
    double time_since_update(rclcpp::Time now) const;
    // 这一帧有没有真收到观测（predict 清零、update 置位）。不用 time_since_update==0：
    // 时间戳重复的消息上那个会误判。
    bool observed_this_frame() const { return observed_this_frame_; }

private:
    rclcpp::Time last_update_time_;
    rclcpp::Time last_time_;
    double dt_;
    bool init_flag_;
    // int id_;
    // 最近 HISTORY_SIZE 帧观测到的身份，-1（这一帧没识别出身份）也占一格。
    std::list<int> history_;

    float sigma_q_x_ = 50.0f;//越小相信模型
    float sigma_q_y_ = 50.0f;
    float sigma_r_x_ = 0.1f;//越小相信观测
    float sigma_r_y_ = 0.1f;

    // 成功关联（update）的次数，用于衡量轨迹成熟度
    int hits_ = 0;

    // 这一帧有没有收到观测。predict 时清零，update 时置位（见 observed_this_frame）。
    bool observed_this_frame_ = false;

    // 全局唯一且稳定的轨迹编号，只用于诊断日志（vector 下标会随着删除变化）
    uint64_t track_id_ = 0;

    // 关联门限的基础半径（m），实际门限为 distance_threshold_ + v * dt_
    double distance_threshold_ = 0.8;

    // 关联代价里对未成熟轨迹的惩罚（m）：让匈牙利优先保住已收敛的轨迹。0 = 纯距离代价。
    double immature_penalty_ = 0.3;

    // 多久没有关联成功就删轨迹（s）。来自 yaml 的 TIME_THRESHOLD，默认同 TIME_THRESHOLD。
    double time_threshold_ = TIME_THRESHOLD;

    // 从零认领一个 id 需要窗口里至少有这么多票。1 票时 1*2 > 1 成立，从单帧垃圾观测建起来
    // 的轨迹第一帧就能认领一个 id，会顶掉"靠冻结位置维持"的那个。来自 yaml 的
    // min_id_evidence。
    int min_id_evidence_ = 3;
};

// 累计诊断计数。以前只在日志文本里、测试没法断言，所以单独留下来。
struct TrackerStats {
    size_t frames = 0;
    size_t obs = 0;                 // 累计观测数
    size_t matched = 0;             // 累计关联成功次数
    size_t spawned = 0;             // 累计新建轨迹数
    size_t deleted = 0;             // 累计删除轨迹数
    size_t deleted_mature_with_id = 0;  // 其中"已经成熟且还认领着一个 id"的（掉追踪的直接代价）
    size_t idnew = 0;
    size_t idsw = 0;                // 同一个 id 换了一条轨迹上报
    size_t idgone = 0;              // 某个 id 这一帧没有任何轨迹能上报（含冻结位置）
    size_t multiid = 0;             // 单条轨迹内部混进了多个 id
    size_t idconflict = 0;          // 同一帧有多条轨迹都能上报同一个 id
    size_t idhold = 0;              // 靠"现任者通吃"上报的帧数（多数表决已经失败）
    // 现任者因为"对这个 id 已经零证据"被夺走的**帧数**（同一次夺走会连续若干帧，所以比
    // "次数"大）。和 idhold 成对：idhold 是持有，idtake 是持有失效后的接手。
    size_t idtake = 0;
    size_t ghost = 0;               // 靠冻结最后位置上报的帧数
    // 冻结位置最陈旧的年龄（帧），用来判断要不要给 blind_hold_time 设上限。
    size_t ghost_max_frames = 0;
};

class TrackerManager {
public:
    TrackerManager();

    radar_msgs::msg::Cars::SharedPtr callback(radar_msgs::msg::Cars::ConstPtr cars);

    // 累计计数，只增不减。用于回归测试和线上体检。
    const TrackerStats& stats() const { return stats_; }

private:
    // 诊断日志统一出口：同时写终端和文件，文件每帧 flush（Ctrl+C 不丢尾部）。
    void debug_out(const std::string& text);

    std::vector<Tracker> trackers_;
    HungarianOptimizer<float> optimizer_;
    radar_msgs::msg::Cars::ConstPtr cars_;

    // 诊断日志开关（debug_tracker）：每帧一行汇总 + 逐事件，定位掉追踪发生在哪一环。
    bool debug_ = false;
    // 诊断日志文件（debug_log_path，开头的 ~ 展开成 $HOME）
    std::string debug_log_path_;
    std::ofstream debug_log_;
    size_t debug_log_bytes_ = 0;
    // 日志体积上限，写满就停，避免跑久了撑爆磁盘
    static constexpr size_t kMaxDebugLogBytes = 64u * 1024u * 1024u;
    // 上一帧每个上报 id 是由哪条轨迹提供的，用于发现 id 掉线/串到别的轨迹
    std::map<int, uint64_t> reported_id_to_track_;

    // 每个 id 当前的归属轨迹（0 = 没有归属，轨迹编号从 1 开始）。归属跟着轨迹活：轨迹还在
    // 别人就夺不走（现任者通吃），轨迹被删才释放。
    std::array<uint64_t, ID_KINDS> id_owner_{};

    // 现任者多久没关联成功就让位（s）。默认 1.5 = TIME_THRESHOLD，轨迹会先被删，所以这条
    // 实际不生效；调小（如 0.5）就回到"跑丢一段时间就把 id 让给别的轨迹"。
    double id_yield_time_ = 1.5;

    // 现任者要"通吃"一个 id，窗口里至少要有这么多票（默认 1）。0 = 永不放手，会在持续性
    // 同 id 误识别下把 id 永久锁死在错误位置上。位置被持续更新不等于身份有证据：场地中央的
    // 假目标票数会衰减到 0，零票时应该让位给能正经认领的轨迹。
    int min_hold_evidence_ = 1;

    // ---- 盲区冻结最后位置 ----
    // 每个 id 最后一次被活轨迹上报的位置。轨迹死掉后只要该 id 还没被重新报出来，就继续发
    // 这个位置（冻结、不外推；带着最多 TIME_THRESHOLD 的滑行误差，但不再增长）。下游拿不到
    // 某个 id 会把坐标写成 (0,0)，而那在赛场坐标系里没法被识别成"无数据"。
    std::array<radar_msgs::msg::Car, ID_KINDS> last_seen_;
    std::array<bool, ID_KINDS> last_seen_valid_{};
    std::array<rclcpp::Time, ID_KINDS> last_seen_stamp_;
    // 这个 id 上一次"由多数表决确认"（而不是靠现任者规则持有）是什么时候，只用于 HOLD 日志
    std::array<rclcpp::Time, ID_KINDS> id_label_stamp_;

    // 冻结位置最多保留多久（s）。-1 = 不设上限；正数则过期作废（IDGONE 这时才真的会响）。
    double blind_hold_time_ = -1.0;

    TrackerStats stats_;
};

#endif