#include <fmt/format.h>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "camera_io/io/camera.hpp"
#include "camera_io/io/cboard.hpp"
#include "tasks/auto_buff/buff_aimer.hpp"
#include "tasks/auto_buff/buff_detector.hpp"
#include "tasks/auto_buff/buff_solver.hpp"
#include "tasks/auto_buff/buff_target.hpp"
#include "tasks/auto_buff/buff_type.hpp"
#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/img_tools.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"
#include "camera_io/tools/plotter.hpp"
#include "camera_io/tools/recorder.hpp"
#include "camera_io/tools/trajectory.hpp"

// 定义命令行参数
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard5.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@input-path    |                        | avi和txt文件的路径}";

double computeIOU(
  int net_width, int net_height, const std::vector<cv::Point2f> & rect1,
  const std::vector<cv::Point2f> & rect2, int fill_value = 255)
{
  cv::Mat img1(net_height, net_width, CV_8UC1, cv::Scalar(0));
  cv::Mat img2(net_height, net_width, CV_8UC1, cv::Scalar(0));
  cv::Mat img3(net_height, net_width, CV_8UC1, cv::Scalar(0));

  std::vector<cv::Point> pts1, pts2;
  for (const auto & pt : rect1) {
    pts1.push_back(cv::Point(pt.x, pt.y));
  }
  for (const auto & pt : rect2) {
    pts2.push_back(cv::Point(pt.x, pt.y));
  }

  cv::fillPoly(img1, std::vector<std::vector<cv::Point>>{pts1}, fill_value);
  cv::fillPoly(img2, std::vector<std::vector<cv::Point>>{pts2}, fill_value);
  cv::fillPoly(img3, std::vector<std::vector<cv::Point>>{pts1}, fill_value);
  cv::fillPoly(img3, std::vector<std::vector<cv::Point>>{pts2}, fill_value);

  int area1 = cv::countNonZero(img1);
  int area2 = cv::countNonZero(img2);
  int area_com = cv::countNonZero(img3);

  double iou = static_cast<double>((area1 + area2 - area_com)) / area_com;
  return iou;
}

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto input_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  // 初始化绘图器和退出器
  tools::Plotter plotter;
  tools::Exiter exiter;

  // 构造视频和文本文件路径
  auto video_path = fmt::format("{}.avi", input_path);
  auto text_path = fmt::format("{}.txt", input_path);
  cv::VideoCapture video(video_path);
  std::ifstream text(text_path);

  // 初始化跟踪器、解算器、追踪器和瞄准器
  auto_buff::Buff_Detector detector(config_path);
  auto_buff::Solver solver(config_path);
  auto_buff::SmallTarget target;
  // auto_buff::BigTarget target;
  auto_buff::Aimer aimer(config_path);

  cv::Mat img;
  auto t0 = std::chrono::steady_clock::now();

  // 设置视频起始帧
  video.set(cv::CAP_PROP_POS_FRAMES, start_index);
  for (int i = 0; i < start_index; i++) {
    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
  }

  std::vector<double> ious;

  // 循环处理视频帧
  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    video.read(img);
    if (img.empty()) break;

    double t, w, x, y, z;
    text >> t >> w >> x >> y >> z;
    auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

    //// 核心逻辑

    solver.set_R_gimbal2world({w, x, y, z});

    // 检测buff
    auto detector_start = std::chrono::steady_clock::now();
    auto power_runes = detector.detect(img);

    // 追踪buff
    auto tracker_start = std::chrono::steady_clock::now();
    solver.solve(power_runes);  // PNP 计算旋转角度

    // 更新目标 瞄准
    auto aimer_start = std::chrono::steady_clock::now();
    target.get_target(power_runes, timestamp);
    auto target_pre = target;
    auto command = aimer.aim(target_pre, timestamp, 27, false);

    /// 调试输出

    auto finish = std::chrono::steady_clock::now();
    tools::logger()->info(
      "[{}] detector: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
      tools::delta_time(tracker_start, detector_start) * 1e3,
      tools::delta_time(aimer_start, tracker_start) * 1e3,
      tools::delta_time(finish, aimer_start) * 1e3);

    nlohmann::json data;

    // buff原始观测数据
    if (power_runes.has_value()) {
      const auto p = power_runes.value();
      data["buff_R_yaw"] = p.ypd_in_world[0] * 57.3;
      data["buff_R_pitch"] = p.ypd_in_world[1] * 57.3;
      data["buff_R_dis"] = p.ypd_in_world[2];

      data["buff_yaw"] = p.ypr_in_world[0] * 57.3;
      data["buff_pitch"] = p.ypr_in_world[1] * 57.3;
      data["buff_roll"] = p.ypr_in_world[2] * 57.3;

      data["buff_R_x"] = p.xyz_in_world[0];
      data["buff_R_y"] = p.xyz_in_world[1];
      data["buff_R_z"] = p.xyz_in_world[2];
      data["buff_R_x2"] = p.blade_xyz_in_world[0];
      data["buff_R_y2"] = p.blade_xyz_in_world[1];
      data["buff_R_z2"] = p.blade_xyz_in_world[2];
    }
    std::vector<cv::Point2f> image_points;
    if (!target.is_unsolve()) {
      auto power_rune = power_runes.value();
      // 显示detect的buff
      for (int i = 0; i < 4; i++) tools::draw_point(img, power_rune.target.points[i], {0, 0, 255});
      tools::draw_point(img, power_rune.target.center, {0, 0, 255}, 3);
      tools::draw_point(img, power_rune.r_center, {0, 0, 255}, 3);

      // 当前帧target更新后buff
      auto Rxyz_in_world_now = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points = solver.reproject_buff(Rxyz_in_world_now, target.ekf_x()[4], target.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {0, 255, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {0, 255, 0});

      // buff瞄准位置(预测)
      double dangle = target.ekf_x()[5] - target_pre.ekf_x()[5];
      auto Rxyz_in_world_pre = target.point_buff2world(Eigen::Vector3d(0.0, 0.0, 0.0));
      image_points =
        solver.reproject_buff(Rxyz_in_world_pre, target_pre.ekf_x()[4], target_pre.ekf_x()[5]);
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4), {255, 0, 0});
      tools::draw_points(
        img, std::vector<cv::Point2f>(image_points.begin() + 4, image_points.end()), {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["R_yaw"] = x[0] * 57.3;
      data["R_V_yaw"] = x[1] * 57.3;
      data["R_pitch"] = x[2] * 57.3;
      data["R_dis"] = x[3];
      data["yaw"] = x[4] * 57.3;

      data["angle"] = x[5] * 57.3;
      data["spd"] = x[6] * 57.3;
      if (x.size() >= 10) {
        data["spd"] = x[6];
        data["a"] = x[7];
        data["w"] = x[8];
        data["fi"] = x[9];
        data["spd0"] = target.spd;
      }
    }

    // 云台响应情况
    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = -ypr[1] * 57.3;

    if (command.control) {
      data["cmd_yaw"] = command.yaw * 57.3;
      data["cmd_pitch"] = command.pitch * 57.3;
    }

    ///

    if (!target.is_unsolve()) {
      // 显示预测buff点位
      double angle = target.ekf_x()[5] - target_pre.ekf_x()[5];
      cv::Point2f pre_in_img =
        solver.point_buff2pixel(cv::Point3f(0.0, 0.7 * std::sin(angle), 0.7 * std::cos(angle)));
      tools::draw_point(img, pre_in_img, {255, 0, 0}, 5);

      // 显示t_pre秒后的buff
      auto t_pre = t + aimer.t_gap;

      std::streampos text_position = text.tellg();  // 保存当前文件指针位置
      int cn = 0;
      while (true) {
        text >> t >> w >> x >> y >> z;  //0.05-0.06
        cn++;
        if (std::abs(t - t_pre) < 0.03 || t > t_pre) break;
      }
      text.seekg(text_position);  // 将文件指针移动到之前保存的位置

      if (std::abs(t - t_pre) < 0.03) {
        double vedio_position = video.get(cv::CAP_PROP_POS_FRAMES);  // 保存当前视频帧的位置
        cv::Mat img_pre;
        while (cn--) video >> img_pre;
        video.set(cv::CAP_PROP_POS_FRAMES, vedio_position);  // 设置视频帧位置到之前保存的位置

        auto_buff::Solver solverpre(config_path);  ///调试用
        solverpre.set_R_gimbal2world({w, x, y, z});
        auto power_runes_pre = detector.detect(img_pre);
        if (power_runes_pre.has_value()) {
          // clang-format off
          auto power_rune_pre = power_runes_pre.value();
          auto R_pre2R = power_runes.value().r_center - power_rune_pre.r_center;
          for (auto & i : power_rune_pre.target.points) i = i + R_pre2R;
          tools::draw_points(img, power_rune_pre.target.points, {0, 0, 255});
          tools::draw_points(img,std::vector<cv::Point2f>{power_rune_pre.target.center + R_pre2R, power_rune_pre.r_center + R_pre2R},{0, 0, 255});
          
          // iou
          auto iou = computeIOU(img.cols, img.rows, power_rune_pre.target.points,std::vector<cv::Point2f>(image_points.begin(), image_points.begin() + 4));
          if (iou > 0) {
            ious.push_back(iou);
            data["iou"] = iou;
            double evg_iou = 0;
            for (double i : ious) evg_iou += i;
            tools::draw_text(img, fmt::format("iou:{:.2f}", evg_iou / ious.size()), power_rune_pre.target.points[0]);
          }
          // clang-format on
        }
      }
    }

    plotter.plot(data);

    cv::resize(img, img, cv::Size(img.size().width * 0.7, img.size().height * 0.7));
    cv::imshow("result", img);

    int key = cv::waitKey(1);
    if (key == 'q') break;
    while (key == ' ') {
      int y = cv::waitKey(30);
      if (y == 'q') break;
    }
  }
  cv::destroyAllWindows();
  text.close();  // 关闭文件

  double mean = 0, variance = 0;
  for (double i : ious) {
    mean += i;
  }
  mean /= ious.size();
  for (double i : ious) {
    variance += std::pow(i - mean, 2);
  }
  variance /= ious.size();
  std::cout << "Mean: " << mean << std::endl;
  std::cout << "Variance: " << variance << std::endl;
  return 0;
}
