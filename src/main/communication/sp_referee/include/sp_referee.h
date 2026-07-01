#pragma once

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/Twist.h>

// #include <XmlRpcValue.h>
#include "serial/serial.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "./data.h"
#include "./check.h"
#include "./protocol.h"

#include "radar_msgs/msg/game_status.hpp"
#include "radar_msgs/msg/game_robot_hp.hpp"
#include "radar_msgs/msg/event_data.hpp"
#include "radar_msgs/msg/robot_status.hpp"
#include "radar_msgs/msg/radar_mark_data.hpp"
#include "radar_msgs/msg/radar_info.hpp"

#include "radar_msgs/msg/radar_cmd.hpp"
#include "radar_msgs/msg/map_robot_data.hpp"
#include "radar_msgs/msg/custom_info.hpp"

#include <std_msgs/msg/int8.hpp>

namespace sp_referee
{
    class Referee :public rclcpp::Node
    {
        public:
            Referee();

            bool init();
            void read();
            void write(uint16_t write_cmd, uint16_t child_cmd = 0);

            // void sendUi();
            // void sendString();

        private:

            // void initCmd(XmlRpc::XmlRpcValue &cmd, std::string type);
            // void initUI(XmlRpc::XmlRpcValue &ui, std::string type);

            int unpack(uint8_t* rx_data);
            void pack(uint8_t* tx_buffer, uint8_t* data, int cmd_id, int len);
            void getRobotInfo();
            void clearRxBuffer();
            void clearTxBuffer();

            // sp_referee::GraphColor getColor(const std::string& color);
            // sp_referee::GraphType getType(const std::string& type);
            // void addUI(UIConfig &config);
            // void updateUI(UIConfig &config);
            // void deleteUI(UIConfig &config);
            // void manipulatorCmdCallback(const sp_common::ManipulatorCmd::ConstPtr &msg);

            void radarCmdCallback(const radar_msgs::msg::RadarCmd::ConstPtr &msg);
            void mapRobotDataCallback(const radar_msgs::msg::MapRobotData::ConstPtr &msg);
            // void mapDataCallback(const sp_referee::MapDataMsgConstPtr &msg);
            void customInfoCallback(const radar_msgs::msg::CustomInfo::ConstPtr &msg);
            // void radarEnemyDartWarningCallback(const std_msgs::msg::Int8::ConstPtr &msg);
            
            // rclcpp::Logger logger_;
            serial::Serial serial_;
            serial::Serial image_transmission_;
            bool use_image_transmission_link_{};
            std::vector<uint8_t> rx_buffer_;
            int rx_len_;
            bool referee_data_is_online_{};
            bool image_trasmission_data_is_online_{};
            // ros::Time last_get_data_time_;
            // ros::Time last_send_data_time_;
            const int frame_length_ = 128, frame_header_length_ = 5, cmd_id_length_ = 2, frame_tail_length_ = 2;
            const int k_unpack_buffer_length_ = 256;
            uint8_t unpack_buffer_[256]{};
            uint8_t tx_buffer_[128]{};
            int tx_len_;
            bool ui_generated{};
            RobotInfo robot_info_;
            Check check_;

            std::vector<uint16_t> read_cmd_;
            std::vector<uint16_t> write_cmd_;

            rclcpp::Publisher<radar_msgs::msg::GameStatus>::SharedPtr game_status_pub_;
            rclcpp::Publisher<radar_msgs::msg::GameRobotHP>::SharedPtr game_robot_hp_pub_;
            rclcpp::Publisher<radar_msgs::msg::EventData>::SharedPtr event_data_pub_;
            rclcpp::Publisher<radar_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
            rclcpp::Publisher<radar_msgs::msg::RadarMarkData>::SharedPtr radar_mark_data_pub_;
            rclcpp::Publisher<radar_msgs::msg::RadarInfo>::SharedPtr radar_info_pub_;
            rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr radar_info_level_pub_;
            
            rclcpp::Subscription<radar_msgs::msg::RadarCmd>::SharedPtr radar_cmd_sub_;
            rclcpp::Subscription<radar_msgs::msg::MapRobotData>::SharedPtr map_robot_data_sub_;
            rclcpp::Subscription<radar_msgs::msg::CustomInfo>::SharedPtr custom_info_sub_;
            // rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr radar_enemy_dart_warning_sub_;

            // ros::Subscriber manipulator_cmd_sub_;
            // sp_common::ManipulatorCmd manipulator_cmd_;
            // radar_msgs::msg::RobotStatus robot_status_ref_;

            radar_msgs::msg::RadarCmd radar_cmd_ref_;
            radar_msgs::msg::MapRobotData map_robot_data_ref_;
            // sp_referee::MapDataMsg map_data_ref_;
            radar_msgs::msg::CustomInfo custom_info_ref_;
            // std_msgs::msg::Int8 radar_enemy_dart_warning_ref_;

            Eigen::Matrix3d last_matrix{};
            Eigen::Matrix3d current_matrix{};
            // ros::Time last_time{};
            // ros::Time current_time{};
            // ros::Publisher velocity_pub_;
            // geometry_msgs::Twist cmd_velocity{};

            enum
            {
                MAUL,
                AUTO,
                JOINT,
                CALI
            };
    };
}
