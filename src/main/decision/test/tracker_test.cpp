// decision 包 tracker 的回归测试。
//
// 用确定性假数据驱动 TrackerManager，断言几条与实现无关的不变式，并把累计指标打印出来供
// 人工对照。config 路径从 argv[1] 拿（CTest 传绝对路径），也可以自己设 DECISION_TRACKER_CONFIG。
//
// 场景（dt = 0.1s，720 帧，7 台车沿 x 往返）：
//   - 6 号车永远不出分类结果，其余随机丢分类，class_id = -1 的观测约占 23%（对齐真实日志）
//   - 3 号车旁跟一个"持续性同 id 误识别"（也报 3、位置差 4m），且 [200,600) 真 3 号车的
//     装甲板全灭（观测照常）→ id 3 必须一帧不掉地报在 3 号车身上
//   - 假目标 1 先带 class_id 1 当上 id 1 的现任者、之后票数归零，真 1 号车回来时必须把
//     id 1 夺回来
//   - 假目标 2 先当上 id 8 的现任者、之后窗口改口认 9，而 8 号车这时回到场上 → id 8 必须
//     回到 8 号车上（不能一边报 9 一边继续占着 8）
//   - 5 号车遮挡 1.0s（< TIME_THRESHOLD，轨迹不会被删）后原地回归 → id 5 接得回来；
//     第 400 帧来一次 80 个观测的爆发 → 不崩，且 3 秒内恢复
//   - [620,680) 4 号车完全无观测（真盲区）→ id 4 一直有上报、位置冻结不动、回来后无缝接回；
//     期间注入一个孤立的带 class_id=4 的垃圾观测 → 顶不掉冻结的 id 4
#include "../include/tracker.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace
{

const double kDt = 0.1;
const double kNoiseSigma = 0.05;  // 观测噪声 std (m)
const int kFrames = 720;
const double kIdMatchRadius = 1.0;  // "报的 id 位置对得上车"的判定半径 (m)

// 3 号车装甲板分类失效的帧区间（观测照常产生）
const int kHoldFrom = 200;
const int kHoldTo = 600;
// 4 号车完全不产生观测的帧区间（轨迹会在 TIME_THRESHOLD 后被删）
const int kGhostFrom = 620;
const int kGhostTo = 680;
// 冻结稳定之后、注入孤立垃圾观测的那一帧
const int kJunk = 660;
// 轨迹（1.5s）+ 一点余量之后，冻结位置必须已经不动的起始帧
const int kFrozenFrom = kGhostFrom + 25;

// ---- 僵尸回收（场地中央的假目标）----
// [0, kZombieLabelTo) 假目标带 class_id=1 当上 id 1 的现任者；之后车体网络照常检出它、
// 但一个标签都不给（票数归零），真 1 号车这时还被 occlude 着 → id 1 靠持有维持；
// [kReclaim, ...) 真 1 号车带标签回归，必须能把 id 1 夺回来。
const int kZombieLabelTo = 150;
const int kReclaim = 300;
const double kZombieX = 16.0;
const double kZombieY = 0.5;

// ---- 窗口被别的 id 占成多数（"污染"）----
// 假目标 2 在 (kPolluteX, kPolluteY)：[0, kPolluteFrom) 报 class_id 8 当上 id 8 的现任者，
// 之后改口报 9（窗口被 9 占成多数，proposal 变成 9，名字上却还挂着 8）；8 号车这时候才
// 回到场上。断言 id 8 回到 8 号车上——改口的轨迹不能继续替旧 id 上报。
const int kPolluteFrom = 60;
const double kPolluteX = 4.0;
const double kPolluteY = 0.5;
const double kPolluteMaxDx = 8.0;  // 挪 8m 就停，别跑出场地
// 8 号车回来后要攒够 min_id_evidence(3) 票才能认领 id 8，中间还会随机丢分类，
// 所以给它 12 帧（1.2s）的宽限；超过这个帧号 id 8 还没回到它身上就算失败。
const int kId8Deadline = kPolluteFrom + 12;

// 场上 7 台车，id 0..6
struct Robot
{
    int id;
    double y;
    double x = 0.0;
    double vx;
    bool always_unlabeled = false;
    int occlude_from = -1;  // [from, to) 帧区间内不产生观测
    int occlude_to = -1;
};

double dist(double ax, double ay, double bx, double by)
{
    return std::sqrt((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}

// 找出输出里某个 id 被报到了哪里；没有则返回 false
bool reported_position(const radar_msgs::msg::Cars& out, int id, double* x, double* y)
{
    for (const auto& car : out.cars) {
        if (car.class_id == id) {
            *x = car.x;
            *y = car.y;
            return true;
        }
    }
    return false;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 1) {
        setenv("DECISION_TRACKER_CONFIG", argv[1], 1);
    }

    // 随机种子可以用 TRK_TEST_SEED 换，方便扫多种丢分类/噪声的组合，别只赌一个种子
    unsigned seed = 20260926u;
    if (const char* s = getenv("TRK_TEST_SEED")) {
        seed = static_cast<unsigned>(strtoul(s, nullptr, 10));
    }

    TrackerManager mgr;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::normal_distribution<double> noise(0.0, kNoiseSigma);

    std::vector<Robot> robots = {
        {0, 2.0, 4.0, 2.0, false, -1, -1},
        {1, 4.0, 20.0, -2.5, false, 0, kReclaim},  // 前 300 帧不在场上，让假目标先占住 id 1
        {2, 6.0, 0.0, 3.0, false, -1, -1},   // 会和其它车对穿
        {3, 2.0, 12.0, -1.5, false, -1, -1},  // 被持续性同 id 误识别盯着 + 长装甲板失效
        {4, 8.0, 22.0, -3.0, false, kGhostFrom, kGhostTo},  // 进真盲区 6s
        {5, 10.0, 6.0, 1.8, false, 150, 160},  // 遮挡 1.0s 后回归
        {6, 12.0, 10.0, 2.2, true, -1, -1},    // 永远没有分类结果
        // 8 号车：前 66 帧不在场上（让假目标 2 先占住 id 8），之后回来把 id 8 要回去
        {8, 13.5, 20.0, -2.0, false, 0, kPolluteFrom + 6},
    };

    int failures = 0;
    int id3_missing_frames = 0;
    int id3_checked_frames = 0;
    int labeled_missing_total = 0;   // car - out 缺口：带 id 的车没被报出来的帧次数
    int labeled_expected_total = 0;
    int id5_recovery_violations = 0;
    int post_burst_violations = 0;
    // ---- 改动 1（现任者持有）----
    int hold_frames = 0;             // 3 号车装甲板失效期间被检查的帧数
    int hold_violations = 0;         // 其中 id 3 掉了 / 报歪了的帧数
    // ---- 改动 2（盲区冻结）----
    int ghost_checked_frames = 0;
    int ghost_missing = 0;           // 盲区里 id 4 没有被补报的帧数
    int ghost_moving = 0;            // 盲区里 id 4 的补报位置还在动的帧数（= 在继续外推）
    int junk_displacement = 0;       // 孤立垃圾观测把冻结位置顶掉的帧数
    int id4_takeover_frames = -1;    // 4 号车回来后 id 4 接回自己身上的帧号（-1 = 没接回）
    // ---- 改动 3（僵尸回收）----
    int zombie_hold_frames = 0;      // 假目标零证据但仍持有 id 1 的帧数（Q2 没被破坏）
    int zombie_hold_violations = 0;  // 其中 id 1 掉了 / 报歪了的帧数
    int zombie_claim_violations = 0; // 假目标有标签期间 id 1 没报在它身上的帧数
    int reclaim_frames = -1;         // 真 1 号车回归后把 id 1 夺回来的帧号（-1 = 没夺回）
    int id8_ok_from = -1;            // 8 号车第一次把 id 8 报在自己身上的帧号（-1 = 没拿回）
    int id8_wrong = 0;               // 8 号车在场、但 id 8 没报在它身上的帧数（污染没被切断）
    int id8_held_frames = 0;         // 假目标 2 改口报 9 之后，id 8 还报在它身上的帧数
    // 上一帧 id 4 被报到的位置，用于判断"冻结"（连续两帧完全一样）
    double id4_prev_x = 0.0, id4_prev_y = 0.0;
    bool id4_prev_valid = false;

    double t = 1000.0;
    for (int f = 0; f < kFrames; ++f) {
        auto msg = std::make_shared<radar_msgs::msg::Cars>();
        msg->header.stamp.sec = static_cast<int32_t>(t);
        msg->header.stamp.nanosec = static_cast<uint32_t>((t - static_cast<int32_t>(t)) * 1e9);

        // ---- 真实车 ----
        std::vector<Robot> labeled_now;
        for (auto& r : robots) {
            r.x += r.vx * kDt;
            if (r.x > 25.0) {
                r.x = 25.0;
                r.vx = -r.vx;
            }
            else if (r.x < 2.0) {
                r.x = 2.0;
                r.vx = -r.vx;
            }

            const bool occluded = f >= r.occlude_from && f < r.occlude_to;
            if (r.occlude_from >= 0 && occluded) {
                continue;
            }

            radar_msgs::msg::Car car;
            car.x = static_cast<float>(r.x + noise(rng));
            car.y = static_cast<float>(r.y + noise(rng));

            // 丢分类：一台车永远没分类结果，3 号车在 [kHoldFrom,kHoldTo) 内装甲板一直
            // 识别不出来（但车体观测照常），其余每帧 11% 概率丢
            // （1/7 + 6/7 * 0.11 ≈ 23%，对齐真实日志实测比例）
            const bool armor_dead = (r.id == 3 && f >= kHoldFrom && f < kHoldTo);
            const bool drop_label = r.always_unlabeled || armor_dead || unit(rng) < 0.11;
            car.class_id = drop_label ? static_cast<int8_t>(-1) : static_cast<int8_t>(r.id);
            if (car.class_id >= 0) {
                labeled_now.push_back(r);
            }
            msg->cars.push_back(car);
        }

        // ---- 持续性同 id 误识别：从第 60 帧起跟着 3 号车，但位置差 4m ----
        //
        // 只在 3 号车还有装甲板证据的时候存在（f < kHoldFrom）。这是故意的：它的每帧
        // 都带 class_id=3 且置信度能攒满，属于"识别层面就错了"，tracker 内部比不过
        // —— 见 min_hold_evidence 的注释。它存在的意义是钉住"现任者通吃"： incumbent
        // 有证据时，一个满票的模仿者照样抢不走 id。等到 3 号车装甲板失效（现任者
        // 零证据）时它必须消失，否则测的就不是 Q2"没人认领时维持 id"，而是"两个都
        // 认领不了 id 的东西谁赢"——那个问题只能在识别层解决。
        if (f >= 60 && f < kHoldFrom) {
            radar_msgs::msg::Car thief;
            thief.class_id = 3;
            thief.x = static_cast<float>(robots[3].x - 4.0 + noise(rng));
            thief.y = static_cast<float>(robots[3].y + noise(rng));
            msg->cars.push_back(thief);
        }

        // ---- 场地中央的假目标：先带标签当上 id 1 的现任者，之后只剩车体观测 ----
        // 位置固定，和任何一台车的航迹都隔着 >3m，门限（0.8m + v*dt）够不着，
        // 所以它一定是自己独立的一条轨迹。
        {
            radar_msgs::msg::Car fake;
            fake.x = static_cast<float>(kZombieX + noise(rng));
            fake.y = static_cast<float>(kZombieY + noise(rng));
            fake.class_id = (f < kZombieLabelTo) ? static_cast<int8_t>(1)
                                                 : static_cast<int8_t>(-1);
            msg->cars.push_back(fake);
        }

        // ---- 假目标 2：窗口被另一个 id 占成多数，但名字上还挂着 id 8 ----
        // 每 10 帧里 7 帧报 9、3 帧报 8 → 窗口稳定在 7 票 9 + 3 票 8：多数是 9
        // （所以它这一帧认领的是 9），但 8 在窗口里还剩 3 票 —— 只要"现任者通吃"
        // 还认这 3 票，它就能一边报 9 一边把 id 8 也按在自己身上。
        {
            radar_msgs::msg::Car fake2;
            const double dx = std::min((f - kPolluteFrom) * 0.1, kPolluteMaxDx);
            fake2.x = static_cast<float>(kPolluteX + std::max(0.0, dx) + noise(rng));
            fake2.y = static_cast<float>(kPolluteY + noise(rng));
            if (f < kPolluteFrom) {
                fake2.class_id = 8;
            }
            else {
                fake2.class_id = ((f - kPolluteFrom) % 10 < 7) ? static_cast<int8_t>(9)
                                                              : static_cast<int8_t>(8);
            }
            msg->cars.push_back(fake2);
        }

        // ---- 第 400 帧的观测爆发：40 个无分类 + 40 个随机 id 的误检 ----
        bool burst_frame = (f == 400);
        if (burst_frame) {
            for (int k = 0; k < 80; ++k) {
                radar_msgs::msg::Car junk;
                junk.x = static_cast<float>(unit(rng) * 27.0);
                junk.y = static_cast<float>(unit(rng) * 14.0);
                junk.class_id = (k < 40) ? static_cast<int8_t>(-1)
                                         : static_cast<int8_t>(7 + (k % 5));
                msg->cars.push_back(junk);
            }
        }

        // ---- 孤立垃圾观测：4 号车已冻结时，来一个带 class_id=4 的单帧误检 ----
        // 放在 x=27（车在 [2,25] 之间往返，门限 0.8+v*dt < 1.2m，够不着）：
        // 它会新建一条 1 帧的轨迹，但 1 票 < min_id_evidence，不该顶掉冻结的 id 4。
        const bool junk_frame = (f == kJunk);
        if (junk_frame) {
            radar_msgs::msg::Car junk;
            junk.x = 27.0f;
            junk.y = 7.0f;
            junk.class_id = 4;
            msg->cars.push_back(junk);
        }

        auto out = mgr.callback(msg);

        // ---- 不变式 1：输出里 id 不重复 ----
        for (size_t a = 0; a < out->cars.size(); ++a) {
            for (size_t b = a + 1; b < out->cars.size(); ++b) {
                if (out->cars[a].class_id == out->cars[b].class_id) {
                    printf("FAIL 帧 %d: 输出里 id=%d 重复\n", f, out->cars[a].class_id);
                    ++failures;
                }
            }
        }

        // ---- 不变式 1b：同一条轨迹不能一帧报两个 id ----
        // 输出里两条记录位置完全相同 = 同一个上报者报了两个 id。它比"id 重复"更隐蔽：
        // id 各自只出现一次，但位置是同一个，下游会当成两台车叠在一起。
        for (size_t a = 0; a < out->cars.size(); ++a) {
            for (size_t b = a + 1; b < out->cars.size(); ++b) {
                if (out->cars[a].x == out->cars[b].x && out->cars[a].y == out->cars[b].y) {
                    printf("FAIL 帧 %d: id=%d 和 id=%d 报在同一个点 (%.3f,%.3f) —— 同一个上报者报了两次\n",
                           f, out->cars[a].class_id, out->cars[b].class_id,
                           out->cars[a].x, out->cars[a].y);
                    ++failures;
                }
            }
        }

        // ---- 不变式 2：带 id 的车必须被报到它自己的位置上（不被误识别抢走 id）----
        // rx/ry 下面会被不变式 3 的循环复用，所以 3 号的坐标单独存一份给不变式 6 用。
        double rx = 0.0, ry = 0.0;
        const bool id3_reported = reported_position(*out, 3, &rx, &ry);
        const double id3_x = rx;
        const double id3_y = ry;
        if (f >= 90 && !burst_frame) {
            ++id3_checked_frames;
            if (!id3_reported) {
                ++id3_missing_frames;  // 3 号车一直可见，正常情况下不该掉
            }
            else if (dist(rx, ry, robots[3].x, robots[3].y) > kIdMatchRadius) {
                printf("FAIL 帧 %d: id 3 报在 (%.2f,%.2f)，3 号车在 (%.2f,%.2f) —— 被误识别抢走了\n",
                       f, rx, ry, robots[3].x, robots[3].y);
                ++failures;
            }
        }

        // ---- 不变式 3：car - out 缺口 ----
        for (const auto& r : labeled_now) {
            ++labeled_expected_total;
            if (!reported_position(*out, r.id, &rx, &ry) ||
                dist(rx, ry, r.x, r.y) > kIdMatchRadius) {
                ++labeled_missing_total;
            }
        }

        // ---- 不变式 4：5 号车遮挡回归后 id 要接回来 ----
        if (f == robots[5].occlude_to + 2) {
            if (!reported_position(*out, 5, &rx, &ry) ||
                dist(rx, ry, robots[5].x, robots[5].y) > kIdMatchRadius) {
                printf("FAIL 帧 %d: 5 号车回归后 id 5 没接回来（报在 %.2f,%.2f，车在 %.2f,%.2f）\n",
                       f, rx, ry, robots[5].x, robots[5].y);
                ++id5_recovery_violations;
                ++failures;
            }
        }

        // ---- 不变式 5：爆发 3 秒后要恢复正常（3 号车仍在，就说明没被误检顶掉）----
        if (f == 430) {
            if (!reported_position(*out, 3, &rx, &ry) ||
                dist(rx, ry, robots[3].x, robots[3].y) > kIdMatchRadius) {
                printf("FAIL 帧 %d: 观测爆发后没恢复\n", f);
                ++post_burst_violations;
                ++failures;
            }
        }

        // ---- 不变式 6（现任者持有）：装甲板失效期间 id 3 必须一帧都不掉 ----
        // 窗口被 -1 填满后多数表决必然失败，这一整段靠的就是"现任者通吃"。
        if (f > kHoldFrom + 13 && f < kHoldTo) {  // 留 13 帧让 -1 把窗口填满
            ++hold_frames;
            if (!id3_reported) {
                printf("FAIL 帧 %d: 装甲板失效期间 id 3 掉了（这条轨迹还在被观测更新）\n", f);
                ++hold_violations;
                ++failures;
            }
            else if (dist(id3_x, id3_y, robots[3].x, robots[3].y) > kIdMatchRadius) {
                printf("FAIL 帧 %d: 装甲板失效期间 id 3 被抢到 (%.2f,%.2f)，车在 (%.2f,%.2f)\n",
                       f, id3_x, id3_y, robots[3].x, robots[3].y);
                ++hold_violations;
                ++failures;
            }
        }

        // ---- 不变式 7（盲区冻结）：真盲区里 id 4 必须一直在,且位置冻结不动 ----
        const bool id4_reported = reported_position(*out, 4, &rx, &ry);
        if (f >= kFrozenFrom && f < kGhostTo) {
            ++ghost_checked_frames;
            if (!id4_reported) {
                printf("FAIL 帧 %d: 4 号车轨迹已被删，id 4 没有被冻结补报\n", f);
                ++ghost_missing;
                ++failures;
            }
            else if (id4_prev_valid && dist(rx, ry, id4_prev_x, id4_prev_y) > 1e-3) {
                // 冻结 = 上一帧报什么这一帧还报什么。还在动说明在继续外推，那就不是冻结。
                printf("FAIL 帧 %d: 冻结位置在动 (%.3f,%.3f) -> (%.3f,%.3f)\n",
                       f, id4_prev_x, id4_prev_y, rx, ry);
                ++ghost_moving;
                ++failures;
            }
            if (f >= kJunk && f <= kJunk + 3 && id4_prev_valid &&
                dist(rx, ry, id4_prev_x, id4_prev_y) > 1e-3) {
                printf("FAIL 帧 %d: 孤立垃圾观测把冻结的 id 4 顶掉了\n", f);
                ++junk_displacement;
                ++failures;
            }
        }
        if (id4_reported) {
            id4_prev_x = rx;
            id4_prev_y = ry;
            id4_prev_valid = true;
        }
        else {
            id4_prev_valid = false;  // 中间断了，下一帧不做连续性判断
        }

        // ---- 不变式 8（冻结无缝接管）：4 号车回来后 id 4 要报回它自己身上 ----
        if (f >= kGhostTo && f < kGhostTo + 25 && id4_takeover_frames < 0) {
            if (id4_reported && dist(rx, ry, robots[4].x, robots[4].y) <= kIdMatchRadius) {
                id4_takeover_frames = f;
            }
        }

        // ---- 不变式 9（僵尸回收）：现任者零证据时，真车必须能把 id 拿回来 ----
        // 放在最后：它要用 rx/ry，而上面的不变式 7/8 还在读这两人份的值。
        const bool id1_reported = reported_position(*out, 1, &rx, &ry);
        if (f >= 30 && f < kZombieLabelTo) {
            // 假目标当上现任者的那一段：id 1 就该报在它身上
            if (!id1_reported || dist(rx, ry, kZombieX, kZombieY) > kIdMatchRadius) {
                printf("FAIL 帧 %d: 假目标还没当上 id 1 的现任者（报在 %.2f,%.2f，期望 %.2f,%.2f）\n",
                       f, rx, ry, kZombieX, kZombieY);
                ++zombie_claim_violations;
                ++failures;
            }
        }
        else if (f >= kZombieLabelTo + 13 && f < kReclaim) {
            // 僵尸段：假目标还在被车体观测更新，但票数归零。没有任何别的轨迹认领 id 1，
            // 所以它必须继续持有——"只要还能持续追踪就一直维护 id"靠这段保证。
            ++zombie_hold_frames;
            if (!id1_reported || dist(rx, ry, kZombieX, kZombieY) > kIdMatchRadius) {
                printf("FAIL 帧 %d: 僵尸段 id 1 没维持在假目标上（报在 %.2f,%.2f）\n", f, rx, ry);
                ++zombie_hold_violations;
                ++failures;
            }
        }
        if (f >= kReclaim && reclaim_frames < 0 && id1_reported &&
            dist(rx, ry, robots[1].x, robots[1].y) <= kIdMatchRadius) {
            reclaim_frames = f;
        }

        // ---- 不变式 10（污染不许延续）：窗口改口认别的 id 之后，旧 id 必须交出去 ----
        // 假目标 2 的多数票已经是 9，认领的只能是 9，名字上挂着的 8 必须放掉。它的旧位置
        // 是冻结的，所以"id 8 还贴着它"只可能是又拿它当了上报者。
        const bool id8_reported = reported_position(*out, 8, &rx, &ry);
        const double fake2_x = kPolluteX + std::max(0.0, std::min((f - kPolluteFrom) * 0.1,
                                                                  kPolluteMaxDx));
        if (id8_reported && f >= kPolluteFrom + 20 &&
            dist(rx, ry, fake2_x, kPolluteY) <= kIdMatchRadius) {
            printf("FAIL 帧 %d: 假目标 2 已经改口认 id 9，却还把 id 8 按在自己身上 (%.2f,%.2f)\n",
                   f, rx, ry);
            ++id8_held_frames;
            ++failures;
        }
        else if (f >= kId8Deadline) {
            // 宽限期之前 8 号车的新轨迹还没攒够票（min_id_evidence=3），允许空着
            // —— 那时 id 8 走冻结补报
            if (!id8_reported) {
                printf("FAIL 帧 %d: 8 号车在场，id 8 却没有上报\n", f);
                ++id8_wrong;
                ++failures;
            }
            else if (dist(rx, ry, robots[7].x, robots[7].y) > kIdMatchRadius) {
                printf("FAIL 帧 %d: id 8 报在 (%.2f,%.2f)，8 号车在 (%.2f,%.2f)\n",
                       f, rx, ry, robots[7].x, robots[7].y);
                ++id8_wrong;
                ++failures;
            }
        }
        if (id8_ok_from < 0 && id8_reported && f >= kPolluteFrom &&
            dist(rx, ry, robots[7].x, robots[7].y) <= kIdMatchRadius) {
            id8_ok_from = f;
        }

        t += kDt;
    }

    const auto& s = mgr.stats();
    printf("\n=== 累计指标（改动前后做对照用）===\n");
    printf("帧数            %zu\n", s.frames);
    printf("观测总数        %zu\n", s.obs);
    printf("关联成功        %zu\n", s.matched);
    printf("新建轨迹 SPAWN  %zu\n", s.spawned);
    printf("删除轨迹 DELETE %zu\n", s.deleted);
    printf("其中成熟且带 id 的删除 %zu  <-- 掉追踪的直接代价\n", s.deleted_mature_with_id);
    printf("IDNEW %zu  IDSW %zu  IDGONE %zu\n", s.idnew, s.idsw, s.idgone);
    printf("MULTIID %zu  IDCONFLICT %zu\n", s.multiid, s.idconflict);
    printf("HOLD %zu（现任者持有上报）  IDTAKE %zu（零证据现任者被夺走）  GHOST %zu（冻结补报）  ghost_max_frames %zu\n",
           s.idhold, s.idtake, s.ghost, s.ghost_max_frames);
    printf("car - out 缺口  %d / %d (%.1f%%)\n", labeled_missing_total, labeled_expected_total,
           labeled_expected_total ? 100.0 * labeled_missing_total / labeled_expected_total : 0.0);
    printf("id 3 掉线帧数   %d / %d\n", id3_missing_frames, id3_checked_frames);
    printf("装甲板失效期间  id 3 异常 %d / %d 帧\n", hold_violations, hold_frames);
    printf("盲区冻结        id 4 漏报 %d / %d 帧，位置在动 %d 帧，被垃圾顶掉 %d 帧\n",
           ghost_missing, ghost_checked_frames, ghost_moving, junk_displacement);
    printf("4 号车回归后 id 4 接回    帧 %d（期望 < %d）\n", id4_takeover_frames, kGhostTo + 25);
    printf("僵尸段 id 1 维持          %d 帧（异常 %d），假目标上位异常 %d\n",
           zombie_hold_frames, zombie_hold_violations, zombie_claim_violations);
    printf("1 号车回归后夺回 id 1     帧 %d（期望 <= %d）\n", reclaim_frames, kReclaim + 8);
    printf("8 号车拿回 id 8           帧 %d（异常 %d 帧），假目标 2 继续占着 id 8 的帧数 %d\n",
           id8_ok_from, id8_wrong, id8_held_frames);

    // 松上界：只用于拦住灾难性回归，具体数值看上面的打印做人工对照
    if (id3_missing_frames > id3_checked_frames / 10) {
        printf("FAIL: id 3 掉线超过 10%%（%d/%d）\n", id3_missing_frames, id3_checked_frames);
        ++failures;
    }
    if (labeled_expected_total > 0 &&
        labeled_missing_total > labeled_expected_total / 10) {
        printf("FAIL: car - out 缺口超过 10%%\n");
        ++failures;
    }
    // 改动 1 / 2 的断言
    if (s.idhold == 0) {
        printf("FAIL: 没有任何一帧是靠现任者持有上报的（装甲板失效那段没被兜住）\n");
        ++failures;
    }
    if (s.ghost == 0) {
        printf("FAIL: 没有任何一帧是冻结补报（盲区里 id 直接消失了）\n");
        ++failures;
    }
    // 盲区冻结不设上限时，一个 id 一旦上过场就再也不会从输出里消失。
    // IDGONE 非零 = 有 id 从输出里掉了，冻结补报的记账写错了。
    if (s.idgone != 0) {
        printf("FAIL: IDGONE=%zu，有 id 从输出里消失了（blind_hold_time=-1 时不该发生）\n", s.idgone);
        ++failures;
    }
    if (id4_takeover_frames < 0) {
        printf("FAIL: 4 号车回归后 id 4 一直没接回自己身上\n");
        ++failures;
    }
    // 改动 3：僵尸回收。这一段是"现任者零证据还占着 id"的直接测试。
    if (zombie_hold_frames == 0) {
        printf("FAIL: 僵尸段一帧都没被检查到（假目标没当上现任者，测试本身失效）\n");
        ++failures;
    }
    if (reclaim_frames < 0 || reclaim_frames > kReclaim + 8) {
        printf("FAIL: 真 1 号车回归后没能在 8 帧内把 id 1 从零证据的僵尸手里夺回来"
               "（reclaim_frames=%d）—— 现任者一直占着不放\n", reclaim_frames);
        ++failures;
    }
    if (s.idtake == 0) {
        printf("FAIL: 一次僵尸回收都没发生（idtake=0），零证据的现任者被永久锁死在错误位置上\n");
        ++failures;
    }
    if (id8_ok_from < 0 || id8_ok_from > kId8Deadline) {
        printf("FAIL: 8 号车没能及时把 id 8 拿回来（帧 %d，期望 <= %d）"
               "—— 污染过的轨迹继续占着旧 id\n", id8_ok_from, kId8Deadline);
        ++failures;
    }
    (void)id5_recovery_violations;
    (void)post_burst_violations;

    printf("\n=== %s（%d 处失败）===\n", failures == 0 ? "PASS" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}
