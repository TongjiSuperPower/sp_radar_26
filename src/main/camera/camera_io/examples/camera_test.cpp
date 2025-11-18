#include "camera_io/io/camera.hpp"

#include <opencv2/opencv.hpp>

#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"

const std::string keys =
    "{help h usage ? |                     | 输出命令行参数说明}"
    "{config-path c  | configs/camera.yaml | yaml配置文件路径 }"
    "{d display      |                     | 显示视频流       }"
    "{o output-video |                     | 输出视频文件路径}"; // 添加视频输出参数

int main(int argc, char *argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help"))
  {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;

  auto config_path = cli.get<std::string>("config-path");
  auto display =true;
  auto output_video = "/home/lp1/RM/radar/tensorrt_yolo_example/media/ouput.mp4";

  io::Camera camera("./src/main/detection/camera_io/configs/camera.yaml");

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();

  // 获取相机的帧率
  double fps = 60;

  // 创建视频写入对象
  cv::VideoWriter video_writer;

  // 使用相机的帧率和视频分辨率
  video_writer.open(output_video, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, cv::Size(3072, 2048));
  if (!video_writer.isOpened())
  {
    tools::logger()->error("无法打开视频文件进行写入: {}", output_video);
    return -1;
  }

  while (!exiter.exit())
  {
    camera.read(img, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);
    tools::logger()->info("img size: {}x{}", img.cols, img.rows);

    // 如果视频输出文件路径不为空，则写入视频文件
    video_writer.write(img); // 写入帧到视频

    if (!display)
      continue;
    cv::imshow("img", img);
    if (cv::waitKey(1) == 'q')
      break;
  }

  // 释放视频写入资源
  if (video_writer.isOpened())
  {
    video_writer.release();
  }

  return 0;
}
