#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "camera_io/io/camera.hpp"
#include "camera_io/io/cboard.hpp"
#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/img_tools.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"
#include "camera_io/tools/plotter.hpp"

using namespace std::chrono_literals;
using namespace std;

int main()
{
  tools::Exiter exiter;
  tools::Plotter plotter;

  io::CBoard cboard("can0");
  io::Camera camera(3, 0.5);

  double cmd_yaw;
  double cmd_pitch;

  cv::Mat img;
  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();
    camera.read(img, timestamp);

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);
    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0);

    auto key = cv::waitKey(1);
    if (key == 'y') {
      cmd_yaw = eulers[0] + 3.14 / 4.0;
      cmd_pitch = eulers[1];
      cboard.send({true, false, cmd_yaw, cmd_pitch});
    }

    if (key == 'p') {
      cmd_yaw = eulers[0];
      cmd_pitch = eulers[1] + 3.14 / 4.0;
      cboard.send({true, false, cmd_yaw, cmd_pitch});
    }

    if (key == 'q') break;

    nlohmann::json data;
    data["yaw"] = eulers[0] * 57.3;
    data["pitch"] = eulers[1] * 57.3;
    plotter.plot(data);

    tools::draw_text(img, "Press y to rotate the yaw axis 45 degrees", {30, 60});
    tools::draw_text(img, "Press p to rotate the pitch axis 45 degrees", {30, 100});
    cv::imshow("camera", img);
  }

  return 0;
}