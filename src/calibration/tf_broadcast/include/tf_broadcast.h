#ifndef SIM_REFEREE_H
#define SIM_REFEREE_H

#include <QWidget>
#include <qtimer.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

namespace Ui
{
    class TfBroadcast;
}

class TfBroadcast : public QWidget
{
    Q_OBJECT

public:
    explicit TfBroadcast(QWidget *parent = nullptr);
    ~TfBroadcast();
    
    void set_transform();
    void init_transform();
public slots:
    void publish_transform();
private slots:
    void on_SpinBox_x_valueChanged(double arg1);
    void on_SpinBox_y_valueChanged(double arg1);
    void on_SpinBox_z_valueChanged(double arg1);
    void on_SpinBox_roll_valueChanged(double arg1);
    void on_SpinBox_pitch_valueChanged(double arg1);
    void on_SpinBox_yaw_valueChanged(double arg1);

private:
    Ui::TfBroadcast *ui;
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::TimerBase::SharedPtr timer_;                         // 定时器用于定时发布
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_; // 用于广播坐标变换
    geometry_msgs::msg::TransformStamped transformStamped_;      // 存储变换信息
    double x_ = 0.0;                                             // 单位mm
    double y_ = 0.0;                                             // 单位mm
    double z_ = 0.0;                                             // 单位mm
    double roll_ = 0.0;                                          // 单位度
    double pitch_ = 0.0;                                         // 单位度
    double yaw_ = 0.0;                                           // 单位度
    QTimer *ros_timer;
};

#endif // SIM_REFEREE_H
