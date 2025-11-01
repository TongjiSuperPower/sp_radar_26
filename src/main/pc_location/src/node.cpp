#include <rclcpp/rclcpp.hpp>
#include <radar_msgs/msg/car_bbox.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
  ImagePublisher() : Node("image_publisher")
  {
    // 创建发布者，使用sensor_msgs/msg/Image消息类型
    publisher_ = this->create_publisher<radar_msgs::msg::CarBbox>("car_bbox", 10);
    
    // 设置定时器，定期发布图像
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 每100ms发布一次
      std::bind(&ImagePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    radar_msgs::msg::CarBbox msg;

    // 设置时间戳
    msg.header.stamp = this->now();
    radar_msgs::msg::Bbox bbox;

    msg.img_height = 960;
    msg.img_width = 1280;

    // for (int i = 0; i < 11; i++) {
    //   bbox.class_confidence = -1;
    //   bbox.class_id = -1;
    //   bbox.x_max = 10;
    //   bbox.x_min = 0;
    //   bbox.y_max = 10;
    //   bbox.y_min = 0;
    //   msg.bboxs.push_back(bbox);
    // }

    bbox.class_confidence = 0.5f;
    bbox.class_id = 10;
    bbox.x_max = 100;
    bbox.x_min = 50;
    bbox.y_max = 100;
    bbox.y_min = 50;

    msg.bboxs.push_back(bbox);
    

    bbox.class_confidence = 0.5f;
    bbox.class_id = 8;
    bbox.x_max = 500;
    bbox.x_min = 450;
    bbox.y_max = 500;
    bbox.y_min = 450;

    msg.bboxs.push_back(bbox);

    // 发布消息
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publishing car bboxes");
  }
  

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<radar_msgs::msg::CarBbox>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}