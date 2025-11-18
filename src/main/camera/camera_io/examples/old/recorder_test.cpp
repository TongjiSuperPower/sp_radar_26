#include "camera_io/tools/recorder.hpp"

#include <opencv2/opencv.hpp>

#include "camera_io/io/camera.hpp"
#include "camera_io/io/cboard.hpp"
#include "camera_io/tools/img_tools.hpp"

const std::string keys =
  "{help h usage ?  |      | 输出命令行参数说明}"
  "{exposure-ms e   |  3.0 | 曝光时间，单位ms }"
  "{gamma g         |  0.5 | 伽马值，基准为1  }"
  "{can c           | can0 | can端口名称     }";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto exposure_ms = cli.get<double>("exposure-ms");
  auto gamma = cli.get<double>("gamma");
  auto can = cli.get<std::string>("can");

  tools::Recorder recorder;
  io::CBoard cboard(can);
  io::Camera camera(exposure_ms, gamma);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  while (true) {
    camera.read(img, timestamp);
    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    recorder.record(img, q, timestamp);

    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q') break;
  }
}