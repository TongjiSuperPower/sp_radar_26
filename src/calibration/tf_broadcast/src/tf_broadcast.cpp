#include "../include/tf_broadcast.h"
#include "ui_tf_broadcast.h"

TfBroadcast::TfBroadcast(QWidget *parent)
    : QWidget(parent), ui(new Ui::TfBroadcast)
{
    ui->setupUi(this);
    node_ = std::make_shared<rclcpp::Node>("tf_broadcast");
    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(publish_transform()));
    ros_timer->start(100); // set the rate to 100ms  You can change this if you want to increase/decrease update rate
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    init_transform();
    set_transform();
    RCLCPP_INFO(node_->get_logger(), "TfBroadcast node has been created.");
}

TfBroadcast::~TfBroadcast()
{
    delete ui;
}
void TfBroadcast::publish_transform()
{
    transformStamped_.header.stamp = node_->get_clock()->now();
    broadcaster_->sendTransform(transformStamped_);
}
void TfBroadcast::set_transform()
{
    transformStamped_.header.frame_id = "lidar_frame";
    transformStamped_.child_frame_id = "camera_frame";
    transformStamped_.transform.translation.x = x_ * 0.001;
    transformStamped_.transform.translation.y = y_ * 0.001;
    transformStamped_.transform.translation.z = z_ * 0.001;
    tf2::Quaternion q;
    q.setRPY(roll_ * M_PI / 180, pitch_ * M_PI / 180, yaw_ * M_PI / 180); // 设置四元数为欧拉角表示
    transformStamped_.transform.rotation.x = q.x();
    transformStamped_.transform.rotation.y = q.y();
    transformStamped_.transform.rotation.z = q.z();
    transformStamped_.transform.rotation.w = q.w();
    RCLCPP_INFO(node_->get_logger(), "Transform has been set.");
}

void TfBroadcast::init_transform()
{
    const auto config = YAML::LoadFile("./src/calibration/tf_broadcast/config/tf_config.yaml");
    x_ = config["tvec_x"].as<double>();
    y_ = config["tvec_y"].as<double>();
    z_ = config["tvec_z"].as<double>();
    yaw_ = config["yaw"].as<double>();
    pitch_ = config["pitch"].as<double>();
    roll_ = config["roll"].as<double>();
    ui->SpinBox_x->setValue(x_);
    ui->SpinBox_y->setValue(y_);
    ui->SpinBox_z->setValue(z_);
    ui->SpinBox_yaw->setValue(yaw_);
    ui->SpinBox_pitch->setValue(pitch_);
    ui->SpinBox_roll->setValue(roll_);
}
void TfBroadcast::on_SpinBox_x_valueChanged(double arg1)
{
    x_ = arg1;
    set_transform();
}

void TfBroadcast::on_SpinBox_y_valueChanged(double arg1)
{
    y_ = arg1;
    set_transform();
}

void TfBroadcast::on_SpinBox_z_valueChanged(double arg1)
{
    z_ = arg1;
    set_transform();
}

void TfBroadcast::on_SpinBox_roll_valueChanged(double arg1)
{
    roll_ = arg1;
    set_transform();
}

void TfBroadcast::on_SpinBox_pitch_valueChanged(double arg1)
{
    pitch_ = arg1;
    set_transform();
}

void TfBroadcast::on_SpinBox_yaw_valueChanged(double arg1)
{
    yaw_ = arg1;
    set_transform();
}
