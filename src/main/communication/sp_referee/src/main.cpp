#include "../include/sp_referee.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto referee_ptr = std::make_shared<sp_referee::Referee>();

  referee_ptr->init();
  while (rclcpp::ok())
  {
    referee_ptr->read();
    rclcpp::spin_some(referee_ptr);

    // referee.sendUi();
    // referee.sendString();
  }

  return 0;
}
