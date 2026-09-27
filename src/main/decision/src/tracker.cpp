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

// 默认是相对工作空间根目录的路径；可以用 DECISION_TRACKER_CONFIG 覆盖，
// 这样从别的目录启动也能读到配置，测试也可以自带一份 yaml 而不动比赛配置。
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
    // yaml 里一直写着 TIME_THRESHOLD，但代码从没读过它（这行被注释掉了），容易误导。
    // 现在真的读，缺省值就是头文件里的 TIME_THRESHOLD。
    time_threshold_ = config["TIME_THRESHOLD"].as<double>(time_threshold_);
    min_id_evidence_ = config["min_id_evidence"].as<int>(min_id_evidence_);
}

void Tracker::update(radar_msgs::msg::Car car, rclcpp::Time stamp)
{
    observed_this_frame_ = true;  // predict 清掉，这里置位（见 observed_this_frame）
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

    // -1 也入队：它是"这一帧没识别出身份"的有效记录，必须占一个窗口槽位，
    // 否则一条持续认错的轨迹可以靠"未识别帧不计入"把自己的置信度撑到 1.0。
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
    // 每帧开头所有轨迹都会 predict 一次，正好用来清"这一帧收到观测没有"的标志。
    // 之后只有被 update 的轨迹会重新置位（见 observed_this_frame）。
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

    // 分母固定 HISTORY_SIZE（不是 history_.size()，也不是以前那个只增不减的计数器），
    // 否则一条只匹配过 1 帧的轨迹会拿到置信度 1.0，反而把成熟轨迹的 id 槽位抢走。
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
        if (id >= 0 && id < ID_KINDS) {  // class_id 越界的观测不参与表决，也挡住了后续越界写
            id_count[id]++;
        }
    }

    const int n = static_cast<int>(history_.size());
    for (int id = 0; id < ID_KINDS; id++) {
        // 两个条件都要满足：多数表决（避免 5/10 这种"一半一半"就算数）+
        // 最低票数（避免窗口里只有 1 帧观测时 1*2 > 1 直接认领一个 id）。
        if (id_count[id] >= min_id_evidence_ && id_count[id] * 2 > n) {
            return id;
        }
    }
    // 没有多数（包括"多数是 -1"）：这一帧不认领任何身份。
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
    // 成熟度：hits_ 越多越可信。新轨迹的代价被抬高，避免匈牙利为了全局最优
    // 把车从一条已经收敛的轨迹手里换给刚建出来的重复轨迹。
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
    // id 归属仲裁和冻结时间的参数，和 debug_ 无关，所以放在 debug 早退之前读
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
    // cars->header.stamp 是 builtin_interfaces::msg::Time，不能直接取 .nanoseconds()
    // （隐式转换只在当参数传的时候发生）。下面几处要按纳秒相减，统一用这个转换好的。
    const rclcpp::Time now(cars->header.stamp);
    const size_t n_trackers = trackers_.size();
    const size_t n_cars = cars_msg->cars.size();
    ++stats_.frames;
    stats_.obs += n_cars;

    // 诊断事件缓冲。事件在处理过程中产生，但汇总行要排在最前面，所以先攒着，
    // 最后一并打印，保证一帧的输出在终端里是连续的一块，方便按帧看。
    std::ostringstream dbg;

    // predict all trackers to the current stamp
    for (auto& tracker : trackers_) {
        tracker.predict(cars->header.stamp);
    }

    // ---- 关联：分两道跑 ----
    // 第一道：带 id 的观测 只配 主导 id 相同的轨迹。这是结构保证——只要真轨迹在门内，
    //        "一辈子没拿到 id 的轨迹"就没有机会抢走它的观测（单帧抢一次就永久上报是
    //        之前掉追踪的根因，见日志）。
    // 第二道：剩下的观测（含全部 class_id == -1 的）对剩下的轨迹，沿用原来的
    //        distance + immature_penalty 代价，保住分类闪断时观测仍能被已有轨迹吸收。
    constexpr float kMaxCost = 1e6f;

    // 每条轨迹当前的主导身份（history_ 的多数表决，见 Tracker::dominant_id）。
    // 关联判据和上报判据必须是同一个函数，否则会出现"关联时认领 X、上报时又不报 X"。
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

    // 在 trk_idx × car_idx 的子矩阵上跑一次匈牙利，行/列下标映射回全局下标。
    // 门外的格子、以及 id_must_match 时身份不符的格子都填哨兵，配上了也在下面丢掉。
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

    // 诊断：区分"观测根本不在门限内"(MISS) 和"在门限内但被别的轨迹抢走"(LOST)。
    // 这两种情况的修法完全相反，所以必须分清楚。此处 trackers_ 还没做删除，
    // 下标和上面几张表仍然对得上。
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
            // 成熟且带着 id 的轨迹被删 = 一台已经在正常上报的机器人彻底没了，
            // 这是"掉追踪"最直接的代价，单独计数。
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

    // 先给每条轨迹算"这一帧想报的 id"。一条轨迹最多提议一个，所以"一条轨迹报两个 id"
    // （MULTIID）在输出路径上结构性地消失；但"一条轨迹内部混了多个 id"仍要能看到，
    // 下面照旧用 get_id_and_confidence() 打诊断。
    //
    // 这里必须用当前的 trackers_.size()，不能用上面那个 n_trackers 快照：那个是在
    // 预测/关联之前取的，之后的建轨迹（push_back）和删轨迹（erase）都会改变 size。
    // size 变小以后再按老快照下标访问就是越界读（读到已被析构的元素，它的 history_
    // 哨兵节点已被释放），日志全开时 heap 复用让这段内存真的不可读，直接段错误。
    const size_t n_out = trackers_.size();
    std::vector<int> proposal(n_out, -1);
    for (size_t i = 0; i < n_out; ++i) {
        const int dom = trackers_[i].dominant_id();
        if (dom >= 0) {
            proposal[i] = dom;
            continue;
        }
        // 多数表决没过（窗口被 -1 填满了）。只要它是某个 id 的现任者就继续替它报：
        // 位置每帧都在被观测更新，身份沿用上一次确认的 —— 这就是"只要还能持续追踪就
        // 一直维护该 id"。真车装甲板几十年看不见（实测最长 41.4s）也不会掉 id。
        //
        // 代价：无标签期间的轨迹对穿交换会粘住 id，粘到轨迹被删为止。带标签的交换
        // 仍然自愈——窗口被新 id 填满后 dominant_id() 就是新 id 了，旧 id 自然释放。
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

    // 没选上现任时，两条候选谁更应该拿这个 id：置信度 -> hits -> track_id（升序）。
    // 最后一级 track_id 必须有：否则完全打平时结果取决于 trackers_ 的遍历顺序，
    // 而 erase 会改变顺序（日志里 id=9 在 trk 86/64 之间反复跳就是这个原因）。
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

        // 现任者在不在。必须在**所有**轨迹里找，不能只在候选里找：现任者的票数掉到
        // 多数门槛以下时它就不是候选了，而那正是下面"持有"规则要兜住的情况
        // （只在候选里找的话，这种帧会被当成"没有现任"，id 直接白送给旁边认领的人）。
        int incumbent = -1;
        if (id_owner_[id] != 0) {
            for (size_t idx = 0; idx < trackers_.size(); ++idx) {
                if (trackers_[idx].track_id() == id_owner_[id]) {
                    incumbent = static_cast<int>(idx);
                    break;
                }
            }
        }

        // 现任者距上次成功关联超过 id_yield_time 就让位，不再参与选举。默认
        // id_yield_time = 1.5（= TIME_THRESHOLD），也就是轨迹会先被删掉，所以这一条
        // 实际不生效 —— 归属一直跟着轨迹活。调小它就回到"跑丢一段时间就把 id 让给
        // 别的轨迹"的老行为。
        const bool incumbent_yields = incumbent >= 0 &&
            trackers_[incumbent].time_since_update(cars->header.stamp) > id_yield_time_;

        // 现任者通吃：只要它还新鲜、窗口里还认得出这个 id，就是它，不比置信度、也不参选。
        //
        // "还认得出"这一条是必须的。位置被车体观测更新（coast=0、hits 每帧涨）不等于
        // 身份有证据：场地中央的一个假目标会被车体网络持续检出（观测都是 class_id=-1），
        // 窗口里的票数一路衰减到 0，但只要它的轨迹还活着，旧规则就让它永远占着 id ——
        // 实测一局里蓝3 被按在场地中央 38s（conf 全程 0），真的蓝3 带着满票申请了 38s
        // 一次都没抢回来；id 7 上同样的事持续了 100s 以上。
        // 所以：票数 >= min_hold_evidence_（默认 1，即最近 1s 内至少有一次确认识别到
        // 这个 id）才通吃；0 票 = 身份完全没有证据，这时只要有别的轨迹能正经认领
        // （多数表决 + min_id_evidence），就该让位。没有任何人认领时它照样继续报，
        // 所以"装甲板看不见但车体还在被跟踪"的 id 维护不受影响。
        //
        // 为什么不干脆比置信度高低："持续性的同 id 误识别"会把自己的票数也攒到 1.0，
        // 比置信度分不出真假。现任者优先（而不是最高票者优先）保证不那么抖；
        // 真正能挡住这种误识别的只有上游 class_confidence（当前仍只记录不拦截）。
        const int incumbent_evidence = incumbent >= 0
            ? trackers_[incumbent].id_count_in_history(id) : 0;
        // proposal[incumbent] == id 这一条保证"一条轨迹一帧最多报一个 id"：现任者的
        // 窗口被别的 id 占成多数时，它这一帧认领的是那个 id（proposal 跟着 majority
        // 走），此时不能再让它顺手把自己名下的旧 id 也报了 —— 那会在输出里出现两条
        // 位置完全相同、id 不同的记录。这时旧 id 交给下面的冻结补报兜着。
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
                // 挑战者必须**这一帧真的收到了观测**，不能靠窗口里的旧票上位。
                // 一条已经跑丢、正等着被删的轨迹窗口里还留着 10 帧旧票（conf=1），
                // 它够得着"认领"的门槛却一帧都没关联上：实测（测试里的持续性误识别）
                // 误识别消失后它的轨迹还苟活 1.5s，正好趁真车零证据的那几帧把 id 抢过去，
                // 然后自己被删 —— id 就被冻结在一个已经死掉的位置上了，一直发到真车
                // 重新认出自己的 id 为止（这局里是 4 秒）。
                if (!trackers_[idx].observed_this_frame()) {
                    continue;
                }
                if (winner < 0 || better(idx, static_cast<size_t>(winner))) {
                    winner = static_cast<int>(idx);
                }
            }
            // 只有让位者一个候选：没人接就继续报，免得白白多出一个空洞
            // （下游会把缺失的 id 发成 (0,0)，那比一个滑行位置更糟）。
            //
            // 但只有在现任者**这一帧认领的就是这个 id** 时才让它兜底（同上，
            // proposal[incumbent] == id）。否则"赢家 ∈ 候选者"这个不变式就断了：
            // 一条窗口已经改口认别的 id 的现任者会顺着这条路把旧 id 也报出去，
            // 两个 id 落在同一个点上。这种情况下旧 id 交给冻结补报。
            if (winner < 0 && incumbent >= 0 && proposal[incumbent] == id) {
                winner = incumbent;
            }
        }
        if (winner < 0) {
            continue;
        }

        // 现任者被抢走了：它是个"活着的僵尸"——轨迹还在被车体观测更新，但身份已经
        // 零证据。这是这次修正的核心事件，必须能被数出来，否则无法判断它是在按预期
        // 工作还是在乱换主。
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

        // 这一帧是靠现任者规则报的（多数表决已经失败）—— 这是"持续追踪就一直维护 id"
        // 的可观测量。没有它这个新行为就无法证伪。
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
        // 上报的置信度是这个 id 在窗口里的票数占比。用 id_count_in_history 而不是
        // best_id_and_confidence，是为了让"现任者靠持有规则保住 id"的那种帧也报出
        // 它真实的票数，而不是多数表决失败时的 0。
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
            // 多数表决已经失败，靠现任者规则继续替它报。这正是"装甲板看不见但车体
            // 还认得出"时维持 id 的那条路，必须能在日志里被数出来。
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
    // 某个 id 的轨迹被删（1.5s 没有观测）之后，如果没有别的活轨迹能报这个 id，就继续
    // 把最后一次上报的位置发出去。冻结、不预测也不外推：位置里带着 TIME_THRESHOLD 的
    // 滑行误差（快车最多 4~5m），但冻结之后这个误差不再增长 —— 这就是"冻结"相对
    // "继续外推"的全部意义。
    //
    // 为什么要补，而不是干脆让它消失：下游 CarsCallback 拿不到的 id 会把坐标写成
    // (0,0)，而 (0,0) 在赛场坐标系里是个看起来完全正常的坐标，接收方分不出"无数据"。
    // 宁可发一个带固定偏差的旧位置。
    //
    // 冻结多久由 blind_hold_time 决定（-1 = 不设上限，只要该 id 没被重新报出来就一直发）。
    // id 被真轨迹重新接管时天然停止补报，不需要别的清理。
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
        // 0 = "冻结"哨兵（轨迹编号从 1 开始）。故意不写 id_owner_[id]：冻结不是归属，
        // 真轨迹回来时照常走上面的选举把 id 接过去。
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

    // 诊断：一条轨迹的历史里混进了多个 class_id，说明它中途抢过别的机器人的观测。
    // 输出路径已经改成一条轨迹只报一个 id，所以这个数看的是轨迹内部的污染程度，
    // 它不再直接变成输出。计数无条件做（has_multiple_ids 只扫一遍 10 格窗口），
    // 只有打印才看 debug_。
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


    // 诊断：对比上一帧，看每个 id 的归属有没有变化。
    // IDSW 是这个 id 换了一条轨迹在报（位置会跳）—— 现在它包含 trk N -> 0（活轨迹被删、
    // 位置由冻结接管）和 trk 0 -> N（真轨迹回来、把冻结位置无缝接过去）两种；
    // IDNEW 是某个 id 第一次出现在输出里；IDGONE 的含义自从有了冻结补报就变了：
    // 不再是"这个 id 没有任何轨迹能上报"，而是"连冻结位置都没有了"（轨迹早已被删且
    // blind_hold_time 过期 / 从没上报过）。所以它应该变得非常稀少。
    // 计数无条件做（就是一张 ≤12 项的 map），只有打印才看 debug_。
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
        // 上游置信度的分布。这一轮只记录、不拦截：先用一局的数据把 min_class_confidence /
        // min_car_confidence 选出来，再在下一轮启用门槛。
        //   cls_*：只统计 class_id >= 0 的观测（它们才有资格写进 history_）
        //   car_*：统计全部观测，用来判断车体得分能不能把"真车体"和"误检框"分开
        // 分桶而不是只报 min/avg，是因为选阈值要看的是低分那一端的形状。
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
              // 本帧有多少个 id 的现任者因为零证据被别的轨迹夺走（僵尸清理）。正常情况下
              // 大部分帧是 0，只在真车从假目标手里拿回 id 的那一帧冒出来。
              << " take=" << take_now
              << " ghost=" << ghost_now
              // 本帧最陈旧的冻结点有多老。不设上限时这是唯一能一眼看出"赛场上留了多久
              // 的假点"的数字，也是事后决定要不要给 blind_hold_time 设上限的依据。
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