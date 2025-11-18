#include "camera_io/io/cboard.hpp"

#include <chrono>
#include <thread>

#include "camera_io/tools/exiter.hpp"
#include "camera_io/tools/logger.hpp"
#include "camera_io/tools/math_tools.hpp"

using namespace std::chrono_literals;

int main()
{
  tools::Exiter exiter;

  io::CBoard cboard("can0");

  while (!exiter.exit()) {
    auto timestamp = std::chrono::steady_clock::now();

    std::this_thread::sleep_for(1ms);

    Eigen::Quaterniond q = cboard.imu_at(timestamp);

    Eigen::Vector3d eulers = tools::eulers(q, 2, 1, 0) * 57.3;
    tools::logger()->info("z{:.2f} y{:.2f} x{:.2f} degree", eulers[0], eulers[1], eulers[2]);
    tools::logger()->info("bullet speed {:.2f} m/s", cboard.bullet_speed);
  }

  return 0;
}