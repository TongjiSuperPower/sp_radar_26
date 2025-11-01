#include "../include/sp_referee.h"
#include "../include/check.h"
#include "../include/data.h"
#include "../include/protocol.h"

namespace sp_referee
{
    void Referee::write(uint16_t write_cmd, uint16_t child_cmd)
    {
        int data_len = 0;
        int frame_len = 0;
        switch (write_cmd)
        {
        case sp_referee::ROBOT_INTERACTIVE_DATA_CMD:
        {
            if (robot_info_.robot_type_ == "radar")
            {
                switch (child_cmd)
                {
                case sp_referee::RADAR_CMD_CMD:
                {
                    data_len = static_cast<int>(sizeof(sp_referee::RadarCmd));
                    frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                    sp_referee::RadarCmd radar_cmd;
                    radar_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_CMD_CMD;
                    radar_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                    radar_cmd.robot_interaction_data_header_.receiver_id_ = 0x8080;
                    radar_cmd.radar_cmd_ = radar_cmd_ref_.radar_cmd;
                    pack(reinterpret_cast<uint8_t *>(&tx_buffer_), reinterpret_cast<uint8_t *>(&radar_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                    break;
                }
                case sp_referee::RADAR_ENEMY_HERO_POSITION_CMD:
                {
                    // data_len = static_cast<int>(sizeof(sp_referee::RadarCmd));
                    // frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                    // sp_referee::RadarCmd radar_cmd;
                    // radar_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_CMD_CMD;
                    // radar_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                    // radar_cmd.robot_interaction_data_header_.receiver_id_ = 0x8080;
                    // radar_cmd.radar_cmd_ = radar_cmd_ref_.radar_cmd;
                    // pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&radar_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                    // break;
                }
                case sp_referee::RADAR_ENEMY_DART_WARNING_CMD:
                {
                    // data_len = static_cast<int>(sizeof(sp_referee::RadarCmd));
                    // frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
                    // sp_referee::RadarCmd radar_cmd;
                    // radar_cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_CMD_CMD;
                    // radar_cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
                    // radar_cmd.robot_interaction_data_header_.receiver_id_ = 0x8080;
                    // radar_cmd.radar_cmd_ = radar_cmd_ref_.radar_cmd;
                    // pack(reinterpret_cast<uint8_t*>(&tx_buffer_), reinterpret_cast<uint8_t*>(&radar_cmd), sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
                    // break;
                }
                }
                break;
            }
            // else if (robot_info_.robot_type_ == "sentry")
            // {
            //     break;
            // }
            // else
            //     break;
        }
        case sp_referee::MAP_ROBOT_DATA_CMD:
        {
            data_len = sizeof(sp_referee::MapRobotData);
            frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::MapRobotData map_robot_data;
            map_robot_data.hero_position_x = map_robot_data_ref_.hero_position_x;
            map_robot_data.hero_position_y = map_robot_data_ref_.hero_position_y;
            map_robot_data.engineer_position_x = map_robot_data_ref_.engineer_position_x;
            map_robot_data.engineer_position_y = map_robot_data_ref_.engineer_position_y;
            map_robot_data.infantry_3_position_x = map_robot_data_ref_.infantry_3_position_x;
            map_robot_data.infantry_3_position_y = map_robot_data_ref_.infantry_3_position_y;
            map_robot_data.infantry_4_position_x = map_robot_data_ref_.infantry_4_position_x;
            map_robot_data.infantry_4_position_y = map_robot_data_ref_.infantry_4_position_y;
            map_robot_data.infantry_5_position_x = map_robot_data_ref_.infantry_5_position_x;
            map_robot_data.infantry_5_position_y = map_robot_data_ref_.infantry_5_position_y;
            map_robot_data.sentry_position_x = map_robot_data_ref_.sentry_position_x;
            map_robot_data.sentry_position_y = map_robot_data_ref_.sentry_position_y;
            pack(tx_buffer_, reinterpret_cast<uint8_t *>(&map_robot_data), sp_referee::MAP_ROBOT_DATA_CMD, data_len);
            break;
        }
        case sp_referee::CUSTOM_INFO_CMD:
        {
            data_len = sizeof(sp_referee::CustomInfo);
            frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
            sp_referee::CustomInfo custom_info;
            custom_info.sender_id_ = custom_info_ref_.sender_id;
            custom_info.receiver_id_ = custom_info_ref_.receiver_id;
            memcpy(custom_info.user_data_, custom_info_ref_.user_data.data(), 30 * sizeof(uint8_t));
            pack(tx_buffer_, reinterpret_cast<uint8_t *>(&custom_info), sp_referee::CUSTOM_INFO_CMD, data_len);
            break;
        }
        default:
        {
            // ROS_WARN("Referee command ID not found.");
            break;
        }
        }
        // last_send_data_time_ = ros::Time::now();

        try
        {
            serial_.write(tx_buffer_, frame_len);
        }
        catch (serial::PortNotOpenedException &e)
        {
        }

        clearTxBuffer();
    }

    void Referee::pack(uint8_t *tx_buffer, uint8_t *data, int cmd_id, int len)
    {
        // memset(tx_buffer, 0, frame_length_);
        auto *frame_header = reinterpret_cast<sp_referee::FrameHeader *>(tx_buffer);
        frame_header->sof_ = 0xA5;
        frame_header->data_length_ = len;
        memcpy(&tx_buffer[frame_header_length_], reinterpret_cast<uint8_t *>(&cmd_id), cmd_id_length_);
        check_.appendCRC8CheckSum(tx_buffer, frame_header_length_);
        memcpy(&tx_buffer[frame_header_length_ + cmd_id_length_], data, len);
        check_.appendCRC16CheckSum(tx_buffer, frame_header_length_ + cmd_id_length_ + len + frame_tail_length_);
        // for (int i = 0; i < frame_header_length_ + cmd_id_length_ + len + frame_tail_length_ ; i++)
        //     ROS_INFO_STREAM(std::hex<<int(tx_buffer[i]));
    }

    void Referee::clearTxBuffer()
    {
        for (int i = 0; i < 128; i++)
            tx_buffer_[i] = 0;
    }

} // namespace sp_referee