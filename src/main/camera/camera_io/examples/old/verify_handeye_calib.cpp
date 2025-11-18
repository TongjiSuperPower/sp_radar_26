#include <fmt/ostream.h>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <thread>
// #include <opencv2/opencv.hpp>

#include "camera_io/io/camera.hpp"
#include "camera_io/io/cboard.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/img_tools.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"
#include "camera_io/tools/plotter.hpp"

using namespace std::chrono_literals;
using namespace cv;
using namespace std;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{can c          | can0 | can端口名称     }"
  "{exposure-ms e  |  3.0 | 曝光时间，单位ms  }"
  "{gamma g        |  0.5 | 伽马值，基准为1   }"
  "{debug d        |      | 输出调试画面和信息 }"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto debug = cli.has("debug");
  auto can = cli.get<std::string>("can");
  auto exposure_ms = cli.get<double>("exposure-ms");
  auto gamma = cli.get<double>("gamma");
  auto config_path = cli.get<std::string>(0);

  io::CBoard cboard(can);
  io::Camera camera(exposure_ms, gamma);

  auto_aim::Classifier classifier(config_path);
  auto_aim::Detector detector(config_path, classifier, debug);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  auto timestamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    camera.read(img, timestamp);

    auto armors = detector.detect(img);

    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);
    solver.set_R_gimbal2world(q);

    for (auto & armor : armors) {
      tools::draw_points(img, armor.points);
      solver.solve(armor);
      auto xyz = armor.xyz_in_world;
      auto ypr = armor.ypr_in_world * 57.3;

      tools::draw_text(
        img, fmt::format("x{:.2f} y{:.2f} z{:.2f}", xyz[0], xyz[1], xyz[2]), {30, 60});
      tools::draw_text(
        img, fmt::format("r{:.1f} p{:.1f} y{:.1f}", ypr[0], ypr[1], ypr[2]), {30, 100});
    }

    cv::imshow("result", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;

  }

  return 0;
}