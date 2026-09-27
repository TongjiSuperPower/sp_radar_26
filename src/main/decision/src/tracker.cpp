#include "../include/tracker.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

// x = [x, vx, y, vy]
// P的初值没细调

namespace
{
constexpr const char* kDefaultConfigPath = "./src/main/decision/config/decision.yaml";

// 默认路径相对工作空间根目录；DECISION_TRACKER_CONFIG 可覆盖（测试用它自带一份 yaml）。
std::string config_path()
{
    const char* env = std::getenv("DECISION_TRACKER_CONFIG");
    if (env != nullptr && env[0] != '\0') {
        return std::string(env);
    }
    return std::string(kDefaultConfigPath);
}

// 全局递增的轨迹编号，只用于诊断日志
uint64_t g_next_track_id = 1;
}  // namespace

Tracker::Tracker() :
    tools::ExtendedKalmanFilter(Eigen::VectorXd::Zero(STATE_SIZE),
        Eigen::MatrixXd::Identity(STATE_SIZE, STATE_SIZE))
{
    init_flag_ = 0;
    track_id_ = g_next_track_id++;
    P(1, 1) *= 100;
    P(3, 3) *= 100;

    auto config = YAML::LoadFile(config_path());
    sigma_q_x_ = config["sigma_q_x"].as<double>();
    sigma_q_y_ = config["sigma_q_y"].as<double>();
    sigma_r_x_ = config["sigma_r_x"].as<double>();
    sigma_r_y_ = config["sigma_r_y"].as<double>();
    distance_threshold_ = config["distance_threshold"].as<double>(distance_threshold_);
    immature_penalty_ = config["immature_penalty"].as<double>(immature_penalty_);
    time_threshold_ = config["TIME_THRESHOLD"].as<double>(time_threshold_);
    min_id_evidence_ = config["min_id_evidence"].as<int>(min_id_evidence_);
}

void Tracker::update(radar_msgs::msg::Car car, rclcpp::Time stamp)
{
    observed_this_frame_ = true;  // predict 时清零，这里置位
    if (init_flag_ == 0) {
        x(0) = car.x;
        x(2) = car.y;  
        init_flag_ = 1;
    }
    else {
        Eigen::VectorXd z = Eigen::VectorXd(MEASUREMENT_SIZE);
        z << car.x, car.y;

        Eigen::MatrixXd R(MEASUREMENT_SIZE, MEASUREMENT_SIZE);
        R << sigma_r_x_,          0, 
                      0, sigma_r_y_;

        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(MEASUREMENT_SIZE, STATE_SIZE);
        H << 1, 0, 0, 0,
             0, 0, 1, 0;

        tools::ExtendedKalmanFilter::update(z, H, R);
    }

    // -1 也占一格：否则持续认错的轨迹能把置信度撑到 1.0。
    history_.push_back(car.class_id);
    if (history_.size() > HISTORY_SIZE) {
        history_.pop_front();
    }

    // if (car.class_id != -1) {
    //     id_ = car.class_id;
    // }
    ++hits_;
    last_update_time_ = stamp;
    last_time_ = stamp;
}

void Tracker::predict(rclcpp::Time now)
{
    // 所有轨迹每帧都会 predict，正好用来清这个标志（update 时置位）。
    observed_this_frame_ = false;
    dt_ = (now.nanoseconds() - last_time_.nanoseconds()) / 1e9;
    last_time_ = now;
    
    Eigen::MatrixXd Q(STATE_SIZE, STATE_SIZE);
    Q << sigma_q_x_*pow(dt_, 3)/3, sigma_q_x_*pow(dt_, 2)/2, 0, 0,
         sigma_q_x_*pow(dt_, 2)/2, sigma_q_x_*pow(dt_, 1), 0, 0,
         0, 0, sigma_q_y_*pow(dt_, 3)/3, sigma_q_y_*pow(dt_, 2)/2,
         0, 0, sigma_q_y_*pow(dt_, 2)/2, sigma_q_y_*pow(dt_, 1);

    // Q << sigma_q_x_, sigma_q_x_, 0, 0,
    //     sigma_q_x_, sigma_q_x_, 0, 0,
    //     0, 0, sigma_q_y_, sigma_q_y_,
    //     0, 0, sigma_q_y_, sigma_q_y_;

    Eigen::MatrixXd F{{1.0, dt_, 0.0, 0.0},
                      {0.0, 1.0, 0.0, 0.0},
                      {0.0, 0.0, 1.0, dt_},
                      {0.0, 0.0, 0.0, 1.0}};

    tools::ExtendedKalmanFilter::predict(F, Q);

}

std::map<int, double> Tracker::get_id_and_confidence()
{
    std::map<int, double> id_and_confidence;

    std::vector<int> id_count(ID_KINDS, 0);
    for (auto id : history_) {
        if (id >= 0 && id < ID_KINDS) {  // Validate class_id
            id_count[id]++;
        }
    }

    // 分母固定 HISTORY_SIZE：用 history_.size() 会让只匹配过 1 帧的轨迹也拿到 1.0。
    for (int id = 0; id < ID_KINDS; id++) {
        if (id_count[id] > 0) {
            id_and_confidence.insert(std::make_pair(id, id_count[id] * 1.0 / HISTORY_SIZE));
        }
    }


    return id_and_confidence;
}

int Tracker::dominant_id() const
{
    if (history_.empty()) {
        return -1;
    }
    std::vector<int> id_count(ID_KINDS, 0);
    for (auto id : history_) {
        if (id >= 0 && id < ID_KINDS) {  // 越界的 class_id 不参与表决
            id_count[id]++;
        }
    }

    const int n = static_cast<int>(history_.size());
    for (int id = 0; id < ID_KINDS; id++) {
        // 多数表决 + 最低票数（防止窗口里只有 1 帧观测就认领一个 id）。
        if (id_count[id] >= min_id_evidence_ && id_count[id] * 2 > n) {
            return id;
        }
    }
    // 没有多数（含"多数是 -1"）就不认领身份。
    return -1;
}

std::pair<int, double> Tracker::best_id_and_confidence() const
{
    const int id = dominant_id();
    if (id < 0) {
        return std::make_pair(-1, 0.0);
    }

    int count = 0;
    for (auto h : history_) {
        if (h == id) {
            count++;
        }
    }
    return std::make_pair(id, count * 1.0 / HISTORY_SIZE);
}

int Tracker::id_count_in_history(int id) const
{
    int count = 0;
    for (auto h : history_) {
        if (h == id) {
            count++;
        }
    }
    return count;
}

bool Tracker::has_multiple_ids() const
{
    uint32_t mask = 0;
    int distinct = 0;
    for (auto id : history_) {
        if (id < 0 || id >= ID_KINDS) {
            continue;
        }
        const uint32_t bit = 1u << id;
        if ((mask & bit) == 0) {
            mask |= bit;
            ++distinct;
        }
    }
    return distinct > 1;
}

std::pair<double, double> Tracker::get_position()
{
    return std::make_pair(x(0), x(2));
}

double Tracker::distance(radar_msgs::msg::Car car)
{ 
    return std::sqrt(std::pow(car.x - x(0), 2) + std::pow(car.y - x(2), 2));
}   

double Tracker::gate_radius() const
{
    const double v = std::sqrt(std::pow(x(1), 2) + std::pow(x(3), 2));
    return distance_threshold_ + v * std::max(dt_, 0.0);
}

double Tracker::time_since_update(rclcpp::Time now) const
{
    return (now.nanoseconds() - last_update_time_.nanoseconds()) / 1e9;
}

bool Tracker::is_near(radar_msgs::msg::Car car)
{
    return distance(car) < gate_radius();
}

double Tracker::association_cost(radar_msgs::msg::Car car)
{
    // 抬高未成熟轨迹的代价，免得匈牙利把车从已收敛的轨迹换给刚建出来的重复轨迹。
    const double maturity = std::min(1.0, hits_ * 1.0 / HISTORY_SIZE);
    return distance(car) + immature_penalty_ * (1.0 - maturity);
}

bool Tracker::has_lost_track(rclcpp::Time now)
{
    int flag = 0;
    if ((now.nanoseconds() - last_update_time_.nanoseconds()) / 1e9 > time_threshold_)
        flag = 1;
    // else if ((x(0) < 1 && x(2) < 1) || (x(0) > 27 && x(2) > 14))  
    //     flag = 1;

    return flag;
}

TrackerManager::TrackerManager()
{
    auto config = YAML::LoadFile(config_path());
    // 这些参数和 debug_ 无关，放在 debug 早退之前读
    id_yield_time_ = config["id_yield_time"].as<double>(id_yield_time_);
    blind_hold_time_ = config["blind_hold_time"].as<double>(blind_hold_time_);
    min_hold_evidence_ = config["min_hold_evidence"].as<int>(min_hold_evidence_);
    debug_ = config["debug_tracker"].as<bool>(debug_);
    if (!debug_) {
        return;
    }

    debug_log_path_ = config["debug_log_path"].as<std::string>("~/decision_run.log");
    if (!debug_log_path_.empty() && debug_log_path_[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home != nullptr) {
            debug_log_path_ = std::string(home) + debug_log_path_.substr(1);
        }
    }

    debug_log_.open(debug_log_path_, std::ios::out | std::ios::trunc);
    if (!debug_log_.is_open()) {
        std::cerr << "[TRK] 打不开日志文件 " << debug_log_path_
                  << "，诊断日志只输出到终端\n";
    }

    // 日志开头写清本次运行的参数，方便事后对比不同配置的日志
    std::ostringstream head;
    head << "[TRK] === decision tracker 诊断日志 ===\n"
         << "[TRK] file=" << debug_log_path_ << "\n"
         << "[TRK] sigma_q_x=" << config["sigma_q_x"].as<double>()
         << " sigma_q_y=" << config["sigma_q_y"].as<double>()
         << " sigma_r_x=" << config["sigma_r_x"].as<double>()
         << " sigma_r_y=" << config["sigma_r_y"].as<double>()
         << " distance_threshold=" << config["distance_threshold"].as<double>(0.8)
         << " immature_penalty=" << config["immature_penalty"].as<double>(0.3)
         << " TIME_THRESHOLD=" << config["TIME_THRESHOLD"].as<double>(TIME_THRESHOLD)
         << " id_yield_time=" << id_yield_time_
         << " blind_hold_time=" << blind_hold_time_ << "(-1=不设上限)"
         << " min_id_evidence=" << config["min_id_evidence"].as<int>(3)
         << " min_hold_evidence=" << min_hold_evidence_ << "(0=现任者永不放手)"
         << " config=" << config_path() << "\n";
    debug_out(head.str());
}

void TrackerManager::debug_out(const std::string& text)
{
    if (debug_log_.is_open()) {
        if (debug_log_bytes_ + text.size() > kMaxDebugLogBytes) {
            debug_log_ << "[TRK] 日志达到 " << kMaxDebugLogBytes
                       << " 字节上限，停止写入文件\n";
            debug_log_.close();
        } else {
            debug_log_ << text;
            debug_log_.flush();  // 每帧 flush：中途 Ctrl+C 也不会丢最后一段
            debug_log_bytes_ += text.size();
        }
    }
    std::cout << text;
}

radar_msgs::msg::Cars::SharedPtr TrackerManager::callback(radar_msgs::msg::Cars::ConstPtr cars)
{
    auto cars_msg = std::make_shared<radar_msgs::msg::Cars>(*cars);
    // header.stamp 是 builtin_interfaces::msg::Time，下面要按纳秒相减，先转一次。
    const rclcpp::Time now(cars->header.stamp);
    const size_t n_trackers = trackers_.size();
    const size_t n_cars = cars_msg->cars.size();
    ++stats_.frames;
    stats_.obs += n_cars;

    // 事件产生在流程中间，但汇总行要排最前，先攒着最后一并打印。
    std::ostringstream dbg;

    // predict all trackers to the current stamp
    for (auto& tracker : trackers_) {
        tracker.predict(cars->header.stamp);
    }

    // ---- 关联：分两道跑 ----
    // 第一道：带 id 的观测只配 dominant_id 相同的轨迹，没 id 的轨迹抢不走真轨迹的观测。
    // 第二道：剩下的观测（含全部 class_id == -1）按距离代价配，兜住分类闪断。
    constexpr float kMaxCost = 1e6f;

    // 关联判据和上报判据都用 dominant_id()，否则会"关联时认领 X、上报时又不报 X"。
    std::vector<int> tracker_id(n_trackers, -1);
    std::vector<bool> has_candidate(n_trackers, false);
    for (size_t i = 0; i < n_trackers; ++i) {
        tracker_id[i] = trackers_[i].dominant_id();
        for (size_t j = 0; j < n_cars; ++j) {
            if (trackers_[i].is_near(cars_msg->cars[j])) {
                has_candidate[i] = true;
                break;
            }
        }
    }

    std::vector<bool> car_matched(n_cars, false);
    std::vector<bool> tracker_matched(n_trackers, false);
    std::vector<int> car_owner(n_cars, -1);  // 诊断用：这个观测最终被哪条轨迹拿走
    size_t n_matched = 0;

    // 在 trk_idx × car_idx 子矩阵上跑匈牙利，下标映射回全局；门外/身份不符的格子填哨兵，
    // 配上后在这里丢掉。
    auto solve = [&](const std::vector<size_t>& trk_idx, const std::vector<size_t>& car_idx,
                     bool id_must_match) {
        if (trk_idx.empty() || car_idx.empty()) {
            return;
        }
        SecureMat<float>* costs = optimizer_.costs();
        costs->Resize(trk_idx.size(), car_idx.size());
        for (size_t r = 0; r < trk_idx.size(); ++r) {
            for (size_t c = 0; c < car_idx.size(); ++c) {
                const size_t i = trk_idx[r];
                const size_t j = car_idx[c];
                const bool id_ok = !id_must_match ||
                    tracker_id[i] == cars_msg->cars[j].class_id;
                if (id_ok && trackers_[i].is_near(cars_msg->cars[j])) {
                    (*costs)(r, c) = static_cast<float>(
                        trackers_[i].association_cost(cars_msg->cars[j]));
                } else {
                    (*costs)(r, c) = kMaxCost;
                }
            }
        }

        std::vector<std::pair<size_t, size_t>> local;
        optimizer_.Minimize(&local);
        for (const auto& [r, c] : local) {
            if (r >= trk_idx.size() || c >= car_idx.size()) {
                continue;  // 方形化用的填充格
            }
            const size_t i = trk_idx[r];
            const size_t j = car_idx[c];
            if (!id_must_match && !trackers_[i].is_near(cars_msg->cars[j])) {
                continue;  // 哨兵格被配上了，丢掉
            }
            if (id_must_match &&
                (tracker_id[i] != cars_msg->cars[j].class_id ||
                 !trackers_[i].is_near(cars_msg->cars[j]))) {
                continue;
            }
            trackers_[i].update(cars_msg->cars[j], cars->header.stamp);
            car_matched[j] = true;
            tracker_matched[i] = true;
            car_owner[j] = static_cast<int>(i);
            ++n_matched;
        }
    };

    std::vector<size_t> stage1_trk;
    std::vector<size_t> stage1_car;
    for (size_t i = 0; i < n_trackers; ++i) {
        if (tracker_id[i] >= 0) {
            stage1_trk.push_back(i);
        }
    }
    for (size_t j = 0; j < n_cars; ++j) {
        const int id = cars_msg->cars[j].class_id;
        if (id >= 0 && id < ID_KINDS) {
            stage1_car.push_back(j);
        }
    }
    solve(stage1_trk, stage1_car, /*id_must_match=*/true);

    std::vector<size_t> stage2_trk;
    std::vector<size_t> stage2_car;
    for (size_t i = 0; i < n_trackers; ++i) {
        if (!tracker_matched[i]) {
            stage2_trk.push_back(i);
        }
    }
    for (size_t j = 0; j < n_cars; ++j) {
        if (!car_matched[j]) {
            stage2_car.push_back(j);
        }
    }
    solve(stage2_trk, stage2_car, /*id_must_match=*/false);
    stats_.matched += n_matched;

    // MISS = 观测不在门限内，LOST = 在门限内被抢走，两种修法相反。此处还没删轨迹，下标对齐。
    if (debug_) {
        for (size_t i = 0; i < n_trackers; ++i) {
            if (tracker_matched[i]) {
                continue;
            }
            const auto pos = trackers_[i].get_position();
            double nearest_d = -1.0;
            size_t nearest_j = 0;
            for (size_t j = 0; j < n_cars; ++j) {
                const double d = trackers_[i].distance(cars_msg->cars[j]);
                if (nearest_d < 0.0 || d < nearest_d) {
                    nearest_d = d;
                    nearest_j = j;
                }
            }
            dbg << "  " << (has_candidate[i] ? "LOST " : "MISS ")
                << " trk=" << trackers_[i].track_id()
                << " hits=" << trackers_[i].hits()
                << " pos=(" << pos.first << "," << pos.second << ")"
                << " v=(" << trackers_[i].x(1) << "," << trackers_[i].x(3) << ")"
                << " gate=" << trackers_[i].gate_radius();
            if (n_cars > 0) {
                dbg << " nearest=#" << nearest_j << " d=" << nearest_d;
                if (has_candidate[i] && car_owner[nearest_j] >= 0) {
                    dbg << " taken_by=trk" << trackers_[car_owner[nearest_j]].track_id();
                }
            }
            dbg << "\n";
        }
    }

    // create a new tracker for every unmatched car
    size_t n_spawned = 0;
    for (size_t j = 0; j < n_cars; ++j) {
        if (car_matched[j]) {
            continue;
        }
        Tracker new_tracker;
        new_tracker.update(cars_msg->cars[j], cars->header.stamp);
        trackers_.push_back(new_tracker);
        ++n_spawned;
        ++stats_.spawned;
        if (debug_) {
            dbg << "  SPAWN  trk=" << new_tracker.track_id()
                << " car=#" << j
                << " pos=(" << cars_msg->cars[j].x << "," << cars_msg->cars[j].y << ")"
                << " id=" << static_cast<int>(cars_msg->cars[j].class_id) << "\n";
        }
    }

    // delete trackers that lose track
    size_t n_deleted = 0;
    for (auto tracker = trackers_.begin(); tracker != trackers_.end(); ) {
        if (tracker->has_lost_track(cars->header.stamp)) {
            // 成熟且带 id 的轨迹被删 = 一台在正常上报的机器人彻底没了，单独计数。
            const bool mature_with_id = tracker->hits() >= HISTORY_SIZE &&
                tracker->best_id_and_confidence().first >= 0;
            if (mature_with_id) {
                ++stats_.deleted_mature_with_id;
            }
            if (debug_) {
                const auto pos = tracker->get_position();
                const auto ids = tracker->get_id_and_confidence();
                dbg << "  DELETE trk=" << tracker->track_id()
                    << " hits=" << tracker->hits()
                    << " coast=" << tracker->time_since_update(cars->header.stamp) << "s"
                    << " pos=(" << pos.first << "," << pos.second << ")"
                    << " ids={";
                for (const auto& [id, conf] : ids) {
                    dbg << " " << id << ":" << conf;
                }
                dbg << " }\n";
            }
            tracker = trackers_.erase(tracker);
            ++n_deleted;
            ++stats_.deleted;
        }
        else {
            tracker++;
        }
    }

    // ---- 输出：每个 id 选举一个上报者 ----
    auto result = std::make_shared<radar_msgs::msg::Cars>();
    std::vector<double> id_confidence(ID_KINDS, 0.0);
    std::map<int, radar_msgs::msg::Car> id_car;
    std::map<int, uint64_t> id_to_track;  // 诊断用：这一帧每个 id 由哪条轨迹上报
    size_t hold_now = 0;                  // 本帧靠现任者规则上报的 id 数（汇总行用）
    size_t take_now = 0;                  // 本帧发生"僵尸清理"（现任者被夺走）的 id 数

    // 每条轨迹提议至多一个 id，所以"一条轨迹报两个 id"在输出路径上结构性地消失。
    //
    // 必须用当前的 trackers_.size()，不能用上面那个快照：建/删轨迹会改变 size，变小后
    // 再按老快照下标访问就是越界读（日志全开时真的段错误过）。
    const size_t n_out = trackers_.size();
    std::vector<int> proposal(n_out, -1);
    for (size_t i = 0; i < n_out; ++i) {
        const int dom = trackers_[i].dominant_id();
        if (dom >= 0) {
            proposal[i] = dom;
            continue;
        }
        // 多数表决没过（窗口被 -1 填满）。只要它还是某个 id 的现任者就继续替它报：
        // 位置每帧都在被更新，身份沿用上次确认的 —— 持续追踪期间 id 就不会掉。
        // 代价：无标签期间的轨迹对穿会粘住 id，粘到轨迹被删。
        for (int id = 0; id < ID_KINDS; ++id) {
            if (id_owner_[id] == trackers_[i].track_id()) {
                proposal[i] = id;
                break;
            }
        }
    }

    std::vector<std::vector<size_t>> candidates(ID_KINDS);
    for (size_t i = 0; i < n_out; ++i) {
        if (proposal[i] >= 0 && proposal[i] < ID_KINDS) {
            candidates[proposal[i]].push_back(i);
        }
    }

    // 候选优先级：置信度 -> hits -> track_id（升序）。最后一级必须有，否则打平时结果取决于
    // 遍历顺序，而 erase 会改变顺序（id 会在两条轨迹间反复跳）。
    auto better = [&](size_t a, size_t b) {
        const double ca = trackers_[a].best_id_and_confidence().second;
        const double cb = trackers_[b].best_id_and_confidence().second;
        if (ca != cb) {
            return ca > cb;
        }
        if (trackers_[a].hits() != trackers_[b].hits()) {
            return trackers_[a].hits() > trackers_[b].hits();
        }
        return trackers_[a].track_id() < trackers_[b].track_id();
    };

    for (int id = 0; id < ID_KINDS; ++id) {
        if (candidates[id].empty()) {
            continue;  // 这一帧没有轨迹能报这个 id（IDGONE 会在下面的诊断里出现）
        }

        // 现任者要在所有轨迹里找，不能只在候选里找：票数掉到门槛以下时它就不是候选了，
        // 而那正是下面"持有"要兜住的情况（否则这种帧会被当成没有现任，id 白送出去）。
        int incumbent = -1;
        if (id_owner_[id] != 0) {
            for (size_t idx = 0; idx < trackers_.size(); ++idx) {
                if (trackers_[idx].track_id() == id_owner_[id]) {
                    incumbent = static_cast<int>(idx);
                    break;
                }
            }
        }

        // 超过 id_yield_time 没关联成功就让位。默认 1.5 = TIME_THRESHOLD，轨迹会先被删，
        // 所以这条实际不生效（归属跟着轨迹活）；调小即回到"跑丢一段时间就换人"。
        const bool incumbent_yields = incumbent >= 0 &&
            trackers_[incumbent].time_since_update(cars->header.stamp) > id_yield_time_;

        // 现任者通吃：新鲜 + 窗口里还认得出这个 id（票数 >= min_hold_evidence，默认 1）。
        // "位置在被观测更新"不等于"身份有证据"：场地中央的假目标会被车体网络持续检出
        // （观测全是 -1），票数衰减到 0 却一直占着 id，真车带满票也抢不回来 —— 零票才让位；
        // 没人认领时它照样继续报。不比置信度：持续性误识别也会把票数攒到 1.0。
        const int incumbent_evidence = incumbent >= 0
            ? trackers_[incumbent].id_count_in_history(id) : 0;
        // proposal[incumbent] == id 保证"一条轨迹一帧最多报一个 id"：现任者窗口被别的 id
        // 占成多数时不能再顺手报旧 id（否则同位置出两条记录），旧 id 交给冻结补报。
        const bool incumbent_holds = incumbent >= 0 && !incumbent_yields &&
            incumbent_evidence >= min_hold_evidence_ &&
            proposal[incumbent] == id;

        int winner = -1;
        if (incumbent_holds) {
            winner = incumbent;
        }
        else {
            for (size_t idx : candidates[id]) {
                if (static_cast<int>(idx) == incumbent) {
                    continue;  // 让位者不参选，否则它靠历史置信度又把自己选回来了
                }
                // 挑战者必须这一帧真的收到观测，不能靠窗口里的旧票上位：一条跑丢等着被删的
                // 轨迹窗口里还留着 10 帧旧票，抢到 id 后自己就被删，id 冻结在死掉的位置上。
                if (!trackers_[idx].observed_this_frame()) {
                    continue;
                }
                if (winner < 0 || better(idx, static_cast<size_t>(winner))) {
                    winner = static_cast<int>(idx);
                }
            }
            // 只有让位者一个候选时继续报，免得白出一个空洞（下游拿到缺失的 id 会发 (0,0)）。
            // 同样要求 proposal[incumbent] == id，否则"赢家 ∈ 候选者"这个不变式会断。
            if (winner < 0 && incumbent >= 0 && proposal[incumbent] == id) {
                winner = incumbent;
            }
        }
        if (winner < 0) {
            continue;
        }

        // 现任者被抢走：它还在被车体观测更新，但身份已零证据。这个事件要能数出来。
        if (incumbent >= 0 && winner != incumbent) {
            ++stats_.idtake;
            ++take_now;
            if (debug_) {
                const double held = id_label_stamp_[id].nanoseconds() > 0
                    ? (now.nanoseconds() - id_label_stamp_[id].nanoseconds()) / 1e9
                    : -1.0;
                const auto old_pos = trackers_[incumbent].get_position();
                const auto new_pos = trackers_[winner].get_position();
                dbg << "  IDTAKE id=" << id << " trk" << trackers_[incumbent].track_id()
                    << "(票=" << incumbent_evidence << " held=" << held << "s"
                    << " hits=" << trackers_[incumbent].hits()
                    << " pos=(" << old_pos.first << "," << old_pos.second << "))"
                    << " -> trk" << trackers_[winner].track_id()
                    << "(票=" << trackers_[winner].id_count_in_history(id)
                    << " hits=" << trackers_[winner].hits()
                    << " pos=(" << new_pos.first << "," << new_pos.second << "))\n";
            }
        }

        // 这一帧靠现任者规则上报（多数表决已失败），HOLD 计数的依据。
        const bool by_hold = trackers_[winner].dominant_id() < 0;

        if (candidates[id].size() > 1) {
            ++stats_.idconflict;
        }
        if (debug_ && candidates[id].size() > 1) {
            dbg << "  IDCONFLICT id=" << id << " 候选 " << candidates[id].size() << " 条:";
            for (size_t idx : candidates[id]) {
                const auto pos = trackers_[idx].get_position();
                dbg << " trk" << trackers_[idx].track_id()
                    << "(conf=" << trackers_[idx].id_count_in_history(id) * 1.0 / HISTORY_SIZE
                    << " hits=" << trackers_[idx].hits()
                    << " coast=" << trackers_[idx].time_since_update(cars->header.stamp)
                    << "s pos=(" << pos.first << "," << pos.second << "))";
            }
            if (winner != incumbent && incumbent >= 0) {
                const auto pos = trackers_[incumbent].get_position();
                dbg << " [现任 trk" << trackers_[incumbent].track_id()
                    << "(conf=" << trackers_[incumbent].id_count_in_history(id) * 1.0 / HISTORY_SIZE
                    << " coast=" << trackers_[incumbent].time_since_update(cars->header.stamp)
                    << "s pos=(" << pos.first << "," << pos.second << "))]";
            }
            dbg << " -> 赢家 trk" << trackers_[winner].track_id()
                << (winner == incumbent ? " (现任保住)"
                                        : (incumbent < 0 ? " (无现任)" : " (换了主)"))
                << (incumbent_yields ? " [现任跑丢超时已让位]" : "") << "\n";
        }

        radar_msgs::msg::Car car;
        car.x = trackers_[winner].get_position().first;
        car.y = trackers_[winner].get_position().second;
        car.class_id = id;
        id_car.insert_or_assign(id, car);
        // 用 id_count_in_history 而不是 best_id_and_confidence：靠持有保住的帧也报真实票数。
        id_confidence[id] = trackers_[winner].id_count_in_history(id) * 1.0 / HISTORY_SIZE;
        id_to_track[id] = trackers_[winner].track_id();
        id_owner_[id] = trackers_[winner].track_id();

        // 记下最后一次"由活轨迹上报"的位置。轨迹被删之后由下面的冻结补报用它。
        last_seen_[id] = car;
        last_seen_valid_[id] = true;
        last_seen_stamp_[id] = cars->header.stamp;
        if (!by_hold) {
            // 上一次"多数表决确认"这个 id 归属的时刻。只为了 HOLD 日志里的 held=。
            id_label_stamp_[id] = cars->header.stamp;
        }
        else {
            // 多数表决已失败，靠现任者规则维持 id —— 装甲板看不见但车体跟得住的那条路。
            ++stats_.idhold;
            ++hold_now;
            if (debug_) {
                const double held = id_label_stamp_[id].nanoseconds() > 0
                    ? (now.nanoseconds() - id_label_stamp_[id].nanoseconds()) / 1e9
                    : -1.0;
                const auto pos = trackers_[winner].get_position();
                dbg << "  HOLD   id=" << id << " trk=" << trackers_[winner].track_id()
                    << " hits=" << trackers_[winner].hits()
                    << " held=" << held << "s"
                    << " conf=" << id_confidence[id]
                    << " coast=" << trackers_[winner].time_since_update(cars->header.stamp) << "s"
                    << " pos=(" << pos.first << "," << pos.second << ")\n";
            }
        }
    }

    // ---- 盲区冻结补报 ----
    // 某个 id 的轨迹被删（1.5s 无观测）后，没有活轨迹能报它就继续发最后的位置：冻结、
    // 不外推（带着最多 4~5m 的滑行误差，但不再增长）。补而不是让它消失，是因为下游拿不到
    // 某个 id 会把坐标写成 (0,0)，而那在赛场坐标系里看着完全正常。
    // 保留多久由 blind_hold_time 决定（-1 = 不限）；真轨迹回来接管时天然停止补报。
    size_t ghost_now = 0;
    double ghost_age_now = 0.0;
    for (int id = 0; id < ID_KINDS; ++id) {
        if (id_car.count(id) > 0 || !last_seen_valid_[id]) {
            continue;
        }
        // 时间戳理论上单调；保险起见钳一下，否则下面的 (size_t) 转换会在时钟回跳时炸掉
        const double age = std::max(
            0.0, (now.nanoseconds() - last_seen_stamp_[id].nanoseconds()) / 1e9);
        if (blind_hold_time_ > 0.0 && age > blind_hold_time_) {
            last_seen_valid_[id] = false;  // 设了上限：过期就作废，免得下一帧又"复活"
            continue;
        }
        radar_msgs::msg::Car car = last_seen_[id];
        car.class_id = id;
        id_car.insert_or_assign(id, car);
        id_confidence[id] = 0.0;
        // 0 = "冻结"哨兵（轨迹编号从 1 开始）；不写 id_owner_，冻结不是归属。
        id_to_track[id] = 0;
        ++stats_.ghost;
        ++ghost_now;
        ghost_age_now = std::max(ghost_age_now, age);
        stats_.ghost_max_frames = std::max(stats_.ghost_max_frames,
                                           static_cast<size_t>(age * 10.0));
        if (debug_) {
            dbg << "  GHOST  id=" << id << " trk=<dead> age=" << age << "s"
                << " pos=(" << car.x << "," << car.y << ")\n";
        }
    }

    // 一条轨迹的历史里混进了多个 class_id = 它中途抢过别人的观测。输出路径已经改成一条
    // 轨迹只报一个 id，所以这个数看的是轨迹内部的污染程度。计数无条件做，打印才看 debug_。
    for (auto& tracker : trackers_) {
        if (!tracker.has_multiple_ids()) {
            continue;
        }
        ++stats_.multiid;
        if (!debug_) {
            continue;
        }
        const auto pos = tracker.get_position();
        const auto id_and_confidences = tracker.get_id_and_confidence();
        dbg << "  MULTIID trk=" << tracker.track_id()
            << " pos=(" << pos.first << "," << pos.second << ") 上报 " << id_and_confidences.size() << " 个 id:";
        for (const auto& [id, conf] : id_and_confidences) {
            dbg << " " << id << ":" << conf;
        }
        dbg << "\n";
    }


    // // ensure each tracker at most outputs one car
    // const double distance_between_car = 0.25;
    // std::vector<int> id_to_be_delete;
    // for (int id_1 = 0; id_1 < 6; id_1++) {
    //     for (int id_2 = id_1 + 1; id_2 < 6; id_2++){
    //         if (pow(id_car[id_1].x - id_car[2].x, 2) && pow(id_car[id_1].y - id_car[id_2].y, 2) 
    //             < pow(distance_between_car, 2)) {
    //                 if (id_confidence[id_1] > id_confidence[id_2])
    //                     id_to_be_delete.push_back(id_2);
    //                 else
    //                     id_to_be_delete.push_back(id_1);
    //         }
    //     }
    // }
    // for (auto id : id_to_be_delete) {
    //     id_car.erase(id);
    // }


    // 对比上一帧看每个 id 的归属变化。IDSW 含 trk N -> 0（轨迹被删、冻结接管）和
    // trk 0 -> N（真轨迹回来接回冻结位置）；IDGONE 现在指"连冻结位置都没了"，应该很稀少。
    for (const auto& [id, track_id] : id_to_track) {
        auto prev = reported_id_to_track_.find(id);
        if (prev == reported_id_to_track_.end()) {
            ++stats_.idnew;
            if (debug_) {
                dbg << "  IDNEW  id=" << id << " trk=" << track_id
                    << " conf=" << id_confidence[id] << "\n";
            }
        }
        else if (prev->second != track_id) {
            ++stats_.idsw;
            if (debug_) {
                dbg << "  IDSW   id=" << id << " trk " << prev->second
                    << " -> " << track_id << " conf=" << id_confidence[id] << "\n";
            }
        }
    }
    for (const auto& [id, track_id] : reported_id_to_track_) {
        if (id_to_track.find(id) == id_to_track.end()) {
            ++stats_.idgone;
            if (debug_) {
                dbg << "  IDGONE id=" << id << " was_trk=" << track_id << "\n";
            }
        }
    }
    reported_id_to_track_ = id_to_track;

    if (debug_) {
        // 上游置信度分布，只记录不拦截：先用一局的数据选 min_class_confidence /
        // min_car_confidence。cls_* 只统计 class_id >= 0 的观测，car_* 统计全部；
        // 分桶是因为选阈值要看低分那一端的形状。
        auto bucket_of = [](float c) -> int {
            if (c <= 0.0f) return 0;
            if (c <= 0.3f) return 1;
            if (c <= 0.6f) return 2;
            if (c <= 0.8f) return 3;
            return 4;
        };
        size_t cls_hist[5] = {0, 0, 0, 0, 0};
        size_t car_hist[5] = {0, 0, 0, 0, 0};
        double cls_min = 0.0, cls_sum = 0.0;
        size_t cls_n = 0;
        double car_min = 0.0, car_sum = 0.0;
        for (size_t j = 0; j < n_cars; ++j) {
            const auto& obs = cars_msg->cars[j];
            if (obs.class_id >= 0) {
                cls_hist[bucket_of(obs.class_confidence)]++;
                if (cls_n == 0 || obs.class_confidence < cls_min) cls_min = obs.class_confidence;
                cls_sum += obs.class_confidence;
                ++cls_n;
            }
            car_hist[bucket_of(obs.car_confidence)]++;
            if (j == 0 || obs.car_confidence < car_min) car_min = obs.car_confidence;
            car_sum += obs.car_confidence;
        }

        std::ostringstream frame;
        frame << "[TRK] t=" << cars->header.stamp.sec << "."
              << cars->header.stamp.nanosec / 1000000
              << " car=" << n_cars
              << " trk=" << trackers_.size()
              << " match=" << n_matched
              << " spawn=" << n_spawned
              << " del=" << n_deleted
              << " out=" << id_car.size()
              << " hold=" << hold_now
              // 本帧被夺走的 id 数（僵尸清理），正常大部分帧是 0。
              << " take=" << take_now
              << " ghost=" << ghost_now
              // 本帧最陈旧的冻结点有多老：判断要不要给 blind_hold_time 设上限的依据。
              << " ghost_max_age=" << ghost_age_now
              << " clsconf_min=" << cls_min
              << " clsconf_avg=" << (cls_n ? cls_sum / cls_n : 0.0)
              << " clsconf_n=" << cls_n
              << " carconf_min=" << car_min
              << " carconf_avg=" << (n_cars ? car_sum / n_cars : 0.0)
              << "\n";
        frame << "  CLSCONF_HIST <=0:" << cls_hist[0] << " 0-0.3:" << cls_hist[1]
              << " 0.3-0.6:" << cls_hist[2] << " 0.6-0.8:" << cls_hist[3]
              << " >0.8:" << cls_hist[4] << "\n";
        frame << "  CARCONF_HIST <=0:" << car_hist[0] << " 0-0.3:" << car_hist[1]
              << " 0.3-0.6:" << car_hist[2] << " 0.6-0.8:" << car_hist[3]
              << " >0.8:" << car_hist[4] << "\n";
        frame << dbg.str();
        debug_out(frame.str());
    }

    for (auto& [id, car] : id_car) {
        result->cars.push_back(car);
    }

    return result;
}