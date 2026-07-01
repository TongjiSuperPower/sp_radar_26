#include "../include/sp_referee.h"

namespace sp_referee
{
    Referee::Referee() : Node("sp_referee")
    {
    }
    bool Referee::init()
    {
        // logger_ = rclcpp::get_logger("com_logger");
        serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(115200);
        serial_.setTimeout(timeout);
        if (serial_.isOpen())
            return false;
        try
        {
            serial_.open();
        }
        catch (serial::IOException &e)
        {
            // RCLCPP_ERROR(logger_, "Cannot open referee port");
        }
        
        // XmlRpc::XmlRpcValue xml_rpc_value;
        // nh_.getParam("read", xml_rpc_value);

        // initCmd(xml_rpc_value, "read");
        // nh_.getParam("write", xml_rpc_value);
        // initCmd(xml_rpc_value, "write");
        // nh_.getParam("ui/mode", xml_rpc_value);
        // initUI(xml_rpc_value, "mode");

        game_status_pub_ = this->create_publisher<radar_msgs::msg::GameStatus>("/game_status", 1);
        game_robot_hp_pub_ = this->create_publisher<radar_msgs::msg::GameRobotHP>("/game_robot_hp", 1);
        event_data_pub_ = this->create_publisher<radar_msgs::msg::EventData>("/event_data", 1);
        robot_status_pub_ = this->create_publisher<radar_msgs::msg::RobotStatus>("/robot_status", 1);
        radar_mark_data_pub_ = this->create_publisher<radar_msgs::msg::RadarMarkData>("/radar_mark_data", 1);
        radar_info_pub_ = this->create_publisher<radar_msgs::msg::RadarInfo>("/radar_info", 1);
        radar_info_level_pub_ = this->create_publisher<std_msgs::msg::Int8>("/radar_info_level", 1);

        // // velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_cc_velocity", 10);
        // // remote_control_pub_ = nh_.advertise<sp_referee::RemoteControlMsg>("/rc_data", 1);

        radar_cmd_sub_ = this->create_subscription<radar_msgs::msg::RadarCmd>(
            "/radar_cmd", 10, std::bind(&Referee::radarCmdCallback, this, std::placeholders::_1));
        map_robot_data_sub_ = this->create_subscription<radar_msgs::msg::MapRobotData>(
            "/map_robot_data", 10, std::bind(&Referee::mapRobotDataCallback, this, std::placeholders::_1));
        // map_data_sub_ = nh_.subscribe<sp_referee::MapDataMsg>("/map_data", 1, &Referee::mapDataCallback, this);
        custom_info_sub_ = this->create_subscription<radar_msgs::msg::CustomInfo>(
            "/custom_info", 10, std::bind(&Referee::customInfoCallback, this, std::placeholders::_1));
        // radar_enemy_dart_warning_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        //     "/dart_gate_status", 10, std::bind(&Referee::radarEnemyDartWarningCallback, this, std::placeholders::_1));
        
        
        RCLCPP_INFO(this->get_logger(), "Referee node has started");

        return true;
    }

    // void Referee::initCmd(XmlRpc::XmlRpcValue &cmd, std::string type)
    // {
    //     if (type == "read")

    //     {
    //         for (int i = 0; i < cmd.size(); ++i)
    //         {
    //             //ROS_INFO_STREAM(std::hex<<static_cast<uint16_t>(static_cast<int>(cmd[i])));
    //             read_cmd_.push_back(static_cast<uint16_t>(static_cast<int>(cmd[i])));
    //         }
    //     }
    //     else if (type == "write")
    //     {

    //         for (int i = 0; i < cmd.size(); ++i)
    //         {
    //             //ROS_INFO_STREAM(std::hex<<static_cast<uint16_t>(static_cast<int>(cmd[i])));
    //             write_cmd_.push_back(static_cast<uint16_t>(static_cast<int>(cmd[i])));
    //         }
    //     }
    //     else
    //     {
    //         ROS_ERROR_STREAM("Illegal type.");
    //         return;
    //     }
    // }

    void Referee::radarCmdCallback(const radar_msgs::msg::RadarCmd::ConstPtr &msg)
    {
        radar_cmd_ref_ = *msg;
        write(0x0301, 0x0121);
    }

    void Referee::mapRobotDataCallback(const radar_msgs::msg::MapRobotData::ConstPtr &msg)
    {
        map_robot_data_ref_ = *msg;
        write(0x0305);
    }

    void Referee::customInfoCallback(const radar_msgs::msg::CustomInfo::ConstPtr &msg)
    {
        custom_info_ref_ = *msg;
        write(0x0308);
    }

    // void Referee::radarEnemyDartWarningCallback(const std_msgs::msg::Int8::ConstPtr &msg)
    // {
    //     radar_enemy_dart_warning_ref_ = *msg;
    //     write(0x0301, 0x0210);
    // }

} // namespace sp_referee