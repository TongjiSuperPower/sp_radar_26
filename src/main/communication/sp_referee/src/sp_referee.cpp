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

        // // velocity_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_cc_velocity", 10);
        // // remote_control_pub_ = nh_.advertise<sp_referee::RemoteControlMsg>("/rc_data", 1);

        radar_cmd_sub_ = this->create_subscription<radar_msgs::msg::RadarCmd>(
            "/radar_cmd", 10, std::bind(&Referee::radarCmdCallback, this, std::placeholders::_1));
        map_robot_data_sub_ = this->create_subscription<radar_msgs::msg::MapRobotData>(
            "/map_robot_data", 10, std::bind(&Referee::mapRobotDataCallback, this, std::placeholders::_1));
        // map_data_sub_ = nh_.subscribe<sp_referee::MapDataMsg>("/map_data", 1, &Referee::mapDataCallback, this);
        custom_info_sub_ = this->create_subscription<radar_msgs::msg::CustomInfo>(
            "/custom_info", 10, std::bind(&Referee::customInfoCallback, this, std::placeholders::_1));
        radar_sentry_position_cmd_sub_ = this->create_subscription<radar_msgs::msg::RadarSentryPositionCmd>(
            "/radar_sentry_position_cmd", 10, std::bind(&Referee::radarSentryPositionCmdCallback, this, std::placeholders::_1));
        combined_data_sub_ = this->create_subscription<radar_msgs::msg::CombinedData>(
            "/combined_data", 10, std::bind(&Referee::combinedDataCallback, this, std::placeholders::_1));
        dart_warning_cmd_sub_ = this->create_subscription<radar_msgs::msg::DartWarningCmd>(
            "/dart_warning_cmd", 10, std::bind(&Referee::dartWarningCmdCallback, this, std::placeholders::_1));
        aerial_countered_cmd_sub_ = this->create_subscription<radar_msgs::msg::AerialCounteredCmd>(
            "/aerial_countered_cmd", 10, std::bind(&Referee::aerialCounteredCmdCallback, this, std::placeholders::_1));

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
        // 直接触发串口写入，receiver固定为裁判系统服务器0x8080
        write(sp_referee::ROBOT_INTERACTIVE_DATA_CMD, sp_referee::RADAR_CMD_CMD, sp_referee::REFEREE_SYSTEM_SERVER);
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

    void Referee::radarSentryPositionCmdCallback(const radar_msgs::msg::RadarSentryPositionCmd::ConstPtr &msg)
    {
        radar_sentry_position_cmd_ref_ = *msg;
        // 只发给己方哨兵7号，直接触发串口写入
        uint16_t sentry_id = (robot_info_.robot_id_ >= 100) ? BLUE_SENTRY : RED_SENTRY;
        write(sp_referee::ROBOT_INTERACTIVE_DATA_CMD, sp_referee::RADAR_SENTRY_POSITION_CMD, sentry_id);
    }

    void Referee::dartWarningCmdCallback(const radar_msgs::msg::DartWarningCmd::ConstPtr &msg)
    {
        // 数据+路由已合并，缓存数据并直接触发串口写入
        radar_enemy_dart_warning_ref_.data = msg->dart_gate_status;
        write(sp_referee::ROBOT_INTERACTIVE_DATA_CMD, sp_referee::RADAR_ENEMY_DART_WARNING_CMD, msg->receiver_id);
    }

    void Referee::aerialCounteredCmdCallback(const radar_msgs::msg::AerialCounteredCmd::ConstPtr &msg)
    {
        // 数据+路由已合并，缓存数据并直接触发串口写入
        radar_aerial_countered_ref_ = *msg;
        write(sp_referee::ROBOT_INTERACTIVE_DATA_CMD, sp_referee::RADAR_AERIAL_COUNTERED_CMD, msg->receiver_id);
    }

    void Referee::combinedDataCallback(const radar_msgs::msg::CombinedData::ConstPtr &msg)
    {
        // 缓存所有数据字段到 radar_ally_combined_ref_
        radar_ally_combined_ref_.hero_hp = msg->hero_hp;
        radar_ally_combined_ref_.engineer_hp = msg->engineer_hp;
        radar_ally_combined_ref_.infantry3_hp = msg->infantry3_hp;
        radar_ally_combined_ref_.infantry4_hp = msg->infantry4_hp;
        radar_ally_combined_ref_.sentry_hp = msg->sentry_hp;
        radar_ally_combined_ref_.hero_ammo = msg->hero_ammo;
        radar_ally_combined_ref_.infantry3_ammo = msg->infantry3_ammo;
        radar_ally_combined_ref_.infantry4_ammo = msg->infantry4_ammo;
        radar_ally_combined_ref_.aerial_ammo = msg->aerial_ammo;
        radar_ally_combined_ref_.sentry_ammo = msg->sentry_ammo;
        radar_ally_combined_ref_.remain_coins = msg->remain_coins;
        radar_ally_combined_ref_.total_coins = msg->total_coins;
        radar_ally_combined_ref_.status_flags = msg->status_flags;
        radar_ally_combined_ref_.hero_heal = msg->hero_heal;
        radar_ally_combined_ref_.hero_cool = msg->hero_cool;
        radar_ally_combined_ref_.hero_def = msg->hero_def;
        radar_ally_combined_ref_.hero_vuln = msg->hero_vuln;
        radar_ally_combined_ref_.hero_atk = msg->hero_atk;
        radar_ally_combined_ref_.engineer_heal = msg->engineer_heal;
        radar_ally_combined_ref_.engineer_cool = msg->engineer_cool;
        radar_ally_combined_ref_.engineer_def = msg->engineer_def;
        radar_ally_combined_ref_.engineer_vuln = msg->engineer_vuln;
        radar_ally_combined_ref_.engineer_atk = msg->engineer_atk;
        radar_ally_combined_ref_.infantry3_heal = msg->infantry3_heal;
        radar_ally_combined_ref_.infantry3_cool = msg->infantry3_cool;
        radar_ally_combined_ref_.infantry3_def = msg->infantry3_def;
        radar_ally_combined_ref_.infantry3_vuln = msg->infantry3_vuln;
        radar_ally_combined_ref_.infantry3_atk = msg->infantry3_atk;
        radar_ally_combined_ref_.infantry4_heal = msg->infantry4_heal;
        radar_ally_combined_ref_.infantry4_cool = msg->infantry4_cool;
        radar_ally_combined_ref_.infantry4_def = msg->infantry4_def;
        radar_ally_combined_ref_.infantry4_vuln = msg->infantry4_vuln;
        radar_ally_combined_ref_.infantry4_atk = msg->infantry4_atk;
        radar_ally_combined_ref_.sentry_heal = msg->sentry_heal;
        radar_ally_combined_ref_.sentry_cool = msg->sentry_cool;
        radar_ally_combined_ref_.sentry_def = msg->sentry_def;
        radar_ally_combined_ref_.sentry_vuln = msg->sentry_vuln;
        radar_ally_combined_ref_.sentry_atk = msg->sentry_atk;
        radar_ally_combined_ref_.sentry_posture = msg->sentry_posture;
        radar_ally_combined_ref_.hero_status = msg->hero_status;
        radar_ally_combined_ref_.engineer_status = msg->engineer_status;
        radar_ally_combined_ref_.infantry3_status = msg->infantry3_status;
        radar_ally_combined_ref_.infantry4_status = msg->infantry4_status;
        radar_ally_combined_ref_.sentry_status = msg->sentry_status;

        // 数据与路由信息已合并，直接触发串口写入
        write(sp_referee::ROBOT_INTERACTIVE_DATA_CMD, msg->data_cmd_id, msg->receiver_id);
    }

} // namespace sp_referee
