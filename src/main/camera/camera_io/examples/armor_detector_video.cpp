#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/detector.hpp"
#include "camera_io/tools/exiter.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml | yaml配置文件的路径}"
  "{start-index s  | 0                      | 视频起始帧下标    }"
  "{end-index e    | 0                      | 视频结束帧下标    }"
  "{@video_path    |                        | avi路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto video_path = cli.get<std::string>(0);
  auto config_path = cli.get<std::string>("config-path");
  auto start_index = cli.get<int>("start-index");
  auto end_index = cli.get<int>("end-index");

  tools::Exiter exiter;

  cv::VideoCapture video(video_path);

  auto_aim::Detector detector(config_path);

  video.set(cv::CAP_PROP_POS_FRAMES, start_index);

  for (int frame_count = start_index; !exiter.exit(); frame_count++) {
    if (end_index > 0 && frame_count > end_index) break;

    cv::Mat img;
    video.read(img);
    if (img.empty()) break;

    auto armors = detector.detect(img, frame_count);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  return 0;
}