# 雷达 0x0301 多机通信子编码协议

> 发送方：雷达（红9 / 蓝109）  
> 通信方式：0x0301（机器人间通信）→ 自定义子编码  
> 数据均为小端序（Little Endian），`__packed` 紧凑排列

---

## 公共帧头 `RobotInteractionDataHeader`（6 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 2 | uint16 | data_cmd_id | 子编码 ID |
| 2 | 2 | uint16 | sender_id | 发送方 ID（红雷达=9，蓝雷达=109） |
| 4 | 2 | uint16 | receiver_id | 接收方 ID |

---

## 子编码列表

| 子编码 | 名称 | 接收方 | 说明 |
|--------|------|--------|------|
| 0x0210 | 敌方飞镖警告 | 全员（除 inf5、哨兵） | 1=敌方飞镖闸门已开启 |
| 0x0211 | 位置信息 | 哨兵 | 含数据来源标志 |
| 0x0212 | HP 信息 | 全员（除 inf5） | 各机器人血量 |
| 0x0213 | 弹药信息 | 全员（除 inf5） | 各机器人剩余弹药 |
| 0x0214 | 场地状态 | 全员（除 inf5） | 金币、状态标志位 |
| 0x0215 | Buff 信息 | 全员（除 inf5） | 全队增益/减益状态 |

---

## 0x0210 — 敌方飞镖警告（8 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 1 | uint8 | dart_gate_status | 1=敌方飞镖闸门已开启 |

---

## 0x0211 — 位置信息（31 字节）

> 仅发哨兵。`source=1` 为信息波数据，`source=0` 为雷达自身定位。单位均为 cm。

| 偏移 | 长度 | 类型 | 字段 | 单位 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 1 | uint8 | source | 0=雷达定位，1=信息波 |
| 7 | 2 | int16 | hero_x | cm |
| 9 | 2 | int16 | hero_y | cm |
| 11 | 2 | int16 | engineer_x | cm |
| 13 | 2 | int16 | engineer_y | cm |
| 15 | 2 | int16 | infantry3_x | cm |
| 17 | 2 | int16 | infantry3_y | cm |
| 19 | 2 | int16 | infantry4_x | cm |
| 21 | 2 | int16 | infantry4_y | cm |
| 23 | 2 | int16 | aerial_x | cm |
| 25 | 2 | int16 | aerial_y | cm |
| 27 | 2 | int16 | sentry_x | cm |
| 29 | 2 | int16 | sentry_y | cm |

---

## 0x0212 — HP 信息（16 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 2 | uint16 | hero_hp | 英雄血量 |
| 8 | 2 | uint16 | engineer_hp | 工程血量 |
| 10 | 2 | uint16 | infantry3_hp | 三号步兵血量 |
| 12 | 2 | uint16 | infantry4_hp | 四号步兵血量 |
| 14 | 2 | uint16 | sentry_hp | 哨兵血量 |

---

## 0x0213 — 弹药信息（16 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 2 | uint16 | hero_ammo | 英雄弹药 |
| 8 | 2 | uint16 | infantry3_ammo | 三号步兵弹药 |
| 10 | 2 | uint16 | infantry4_ammo | 四号步兵弹药 |
| 12 | 2 | uint16 | aerial_ammo | 空中机器人弹药 |
| 14 | 2 | uint16 | sentry_ammo | 哨兵弹药 |

---

## 0x0214 — 场地状态（14 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 2 | uint16 | remain_coins | 剩余金币 |
| 8 | 2 | uint16 | total_coins | 总金币 |
| 10 | 4 | uint32 | status_flags | 场地状态标志位 |

---

## 0x0215 — Buff 信息（43 字节）

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 1 | uint8 | hero_heal | 英雄回血 |
| 7 | 2 | uint16 | hero_cool | 英雄冷却 |
| 9 | 1 | uint8 | hero_def | 英雄防御 |
| 10 | 1 | uint8 | hero_vuln | 英雄易伤 |
| 11 | 2 | uint16 | hero_atk | 英雄攻击 |
| 13 | 1 | uint8 | engineer_heal | 工程回血 |
| 14 | 2 | uint16 | engineer_cool | 工程冷却 |
| 16 | 1 | uint8 | engineer_def | 工程防御 |
| 17 | 1 | uint8 | engineer_vuln | 工程易伤 |
| 18 | 2 | uint16 | engineer_atk | 工程攻击 |
| 20 | 1 | uint8 | infantry3_heal | 步兵3回血 |
| 21 | 2 | uint16 | infantry3_cool | 步兵3冷却 |
| 23 | 1 | uint8 | infantry3_def | 步兵3防御 |
| 24 | 1 | uint8 | infantry3_vuln | 步兵3易伤 |
| 25 | 2 | uint16 | infantry3_atk | 步兵3攻击 |
| 27 | 1 | uint8 | infantry4_heal | 步兵4回血 |
| 28 | 2 | uint16 | infantry4_cool | 步兵4冷却 |
| 30 | 1 | uint8 | infantry4_def | 步兵4防御 |
| 31 | 1 | uint8 | infantry4_vuln | 步兵4易伤 |
| 32 | 2 | uint16 | infantry4_atk | 步兵4攻击 |
| 34 | 1 | uint8 | sentry_heal | 哨兵回血 |
| 35 | 2 | uint16 | sentry_cool | 哨兵冷却 |
| 37 | 1 | uint8 | sentry_def | 哨兵防御 |
| 38 | 1 | uint8 | sentry_vuln | 哨兵易伤 |
| 39 | 2 | uint16 | sentry_atk | 哨兵攻击 |
| 41 | 1 | uint8 | sentry_posture | 哨兵姿态 |
| 42 | 1 | uint8 | hero_status | 英雄存活状态 |
| 43 | 1 | uint8 | engineer_status | 工程存活状态 |
| 44 | 1 | uint8 | infantry3_status | 步兵3存活状态 |
| 45 | 1 | uint8 | infantry4_status | 步兵4存活状态 |
| 46 | 1 | uint8 | sentry_status | 哨兵存活状态 |

---

## 机器人 ID 对照表

| 机器人 | 红方 ID | 蓝方 ID |
|--------|---------|---------|
| 英雄 | 1 | 101 |
| 工程 | 2 | 102 |
| 步兵3 | 3 | 103 |
| 步兵4 | 4 | 104 |
| ~~步兵5~~ | ~~5~~ | ~~105~~ |
| 空中 | 6 | 106 |
| 哨兵 | 7 | 107 |
| 飞镖 | 8 | 108 |
| **雷达** | **9** | **109** |

---

## 发送端代码

```cpp
case sp_referee::RADAR_ENEMY_DART_WARNING_CMD:
{
    int base = (robot_info_.robot_id_ >= 100) ? 101 : 1;
    for (int i = base; i < base + 7; ++i) {
        if (i == base + 4 || i == base + 6) continue; // skip inf5 and sentry
        data_len = static_cast<int>(sizeof(sp_referee::RadarEnemyDartWarning));
        frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
        sp_referee::RadarEnemyDartWarning cmd;
        cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ENEMY_DART_WARNING_CMD;
        cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
        cmd.robot_interaction_data_header_.receiver_id_ = i;
        cmd.dart_gate_status_ = radar_enemy_dart_warning_ref_.data;
        pack(reinterpret_cast<uint8_t *>(&tx_buffer_),
             reinterpret_cast<uint8_t *>(&cmd),
             sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
    }
    break;
}

case sp_referee::RADAR_SENTRY_POSITION_CMD:
{
    data_len = static_cast<int>(sizeof(sp_referee::RadarSentryPositionCmd));
    frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
    sp_referee::RadarSentryPositionCmd cmd;
    cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_SENTRY_POSITION_CMD;
    cmd.robot_interaction_data_header_.sender_id_ = robot_info_.robot_id_;
    cmd.robot_interaction_data_header_.receiver_id_ =
        (robot_info_.robot_id_ >= 100) ? sp_referee::BLUE_SENTRY : sp_referee::RED_SENTRY;
    cmd.source_          = radar_sentry_position_cmd_ref_.source;
    cmd.hero_x_          = radar_sentry_position_cmd_ref_.hero_x;
    cmd.hero_y_          = radar_sentry_position_cmd_ref_.hero_y;
    cmd.engineer_x_      = radar_sentry_position_cmd_ref_.engineer_x;
    cmd.engineer_y_      = radar_sentry_position_cmd_ref_.engineer_y;
    cmd.infantry3_x_     = radar_sentry_position_cmd_ref_.infantry3_x;
    cmd.infantry3_y_     = radar_sentry_position_cmd_ref_.infantry3_y;
    cmd.infantry4_x_     = radar_sentry_position_cmd_ref_.infantry4_x;
    cmd.infantry4_y_     = radar_sentry_position_cmd_ref_.infantry4_y;
    cmd.aerial_x_        = radar_sentry_position_cmd_ref_.aerial_x;
    cmd.aerial_y_        = radar_sentry_position_cmd_ref_.aerial_y;
    cmd.sentry_x_        = radar_sentry_position_cmd_ref_.sentry_x;
    cmd.sentry_y_        = radar_sentry_position_cmd_ref_.sentry_y;
    pack(reinterpret_cast<uint8_t *>(&tx_buffer_),
         reinterpret_cast<uint8_t *>(&cmd),
         sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
    break;
}

case sp_referee::RADAR_ALLY_HP_CMD:
{
    int base = (robot_info_.robot_id_ >= 100) ? 101 : 1;
    for (int i = base; i < base + 7; ++i) {
        if (i == base + 4) continue; // skip inf5
        data_len = static_cast<int>(sizeof(sp_referee::RadarAllyHpCmd));
        frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
        sp_referee::RadarAllyHpCmd cmd;
        cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ALLY_HP_CMD;
        cmd.robot_interaction_data_header_.sender_id_    = robot_info_.robot_id_;
        cmd.robot_interaction_data_header_.receiver_id_  = i;
        cmd.hero_hp_      = radar_ally_hp_cmd_ref_.hero_hp;
        cmd.engineer_hp_  = radar_ally_hp_cmd_ref_.engineer_hp;
        cmd.infantry3_hp_ = radar_ally_hp_cmd_ref_.infantry3_hp;
        cmd.infantry4_hp_ = radar_ally_hp_cmd_ref_.infantry4_hp;
        cmd.sentry_hp_    = radar_ally_hp_cmd_ref_.sentry_hp;
        pack(reinterpret_cast<uint8_t *>(&tx_buffer_),
             reinterpret_cast<uint8_t *>(&cmd),
             sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
    }
    break;
}

case sp_referee::RADAR_ALLY_AMMO_CMD:
{
    int base = (robot_info_.robot_id_ >= 100) ? 101 : 1;
    for (int i = base; i < base + 7; ++i) {
        if (i == base + 4) continue;
        data_len = static_cast<int>(sizeof(sp_referee::RadarAllyAmmoCmd));
        frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
        sp_referee::RadarAllyAmmoCmd cmd;
        cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ALLY_AMMO_CMD;
        cmd.robot_interaction_data_header_.sender_id_    = robot_info_.robot_id_;
        cmd.robot_interaction_data_header_.receiver_id_  = i;
        cmd.hero_ammo_      = radar_ally_ammo_cmd_ref_.hero_ammo;
        cmd.infantry3_ammo_ = radar_ally_ammo_cmd_ref_.infantry3_ammo;
        cmd.infantry4_ammo_ = radar_ally_ammo_cmd_ref_.infantry4_ammo;
        cmd.aerial_ammo_    = radar_ally_ammo_cmd_ref_.aerial_ammo;
        cmd.sentry_ammo_    = radar_ally_ammo_cmd_ref_.sentry_ammo;
        pack(reinterpret_cast<uint8_t *>(&tx_buffer_),
             reinterpret_cast<uint8_t *>(&cmd),
             sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
    }
    break;
}

case sp_referee::RADAR_ALLY_FIELD_CMD:
{
    int base = (robot_info_.robot_id_ >= 100) ? 101 : 1;
    for (int i = base; i < base + 7; ++i) {
        if (i == base + 4) continue;
        data_len = static_cast<int>(sizeof(sp_referee::RadarAllyFieldCmd));
        frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
        sp_referee::RadarAllyFieldCmd cmd;
        cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ALLY_FIELD_CMD;
        cmd.robot_interaction_data_header_.sender_id_    = robot_info_.robot_id_;
        cmd.robot_interaction_data_header_.receiver_id_  = i;
        cmd.remain_coins_ = radar_ally_field_cmd_ref_.remain_coins;
        cmd.total_coins_  = radar_ally_field_cmd_ref_.total_coins;
        cmd.status_flags_ = radar_ally_field_cmd_ref_.status_flags;
        pack(reinterpret_cast<uint8_t *>(&tx_buffer_),
             reinterpret_cast<uint8_t *>(&cmd),
             sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
    }
    break;
}

case sp_referee::RADAR_ALLY_BUFF_CMD:
{
    int base = (robot_info_.robot_id_ >= 100) ? 101 : 1;
    for (int i = base; i < base + 7; ++i) {
        if (i == base + 4) continue;
        data_len = static_cast<int>(sizeof(sp_referee::RadarAllyBuffCmd));
        frame_len = frame_header_length_ + cmd_id_length_ + data_len + frame_tail_length_;
        sp_referee::RadarAllyBuffCmd cmd;
        cmd.robot_interaction_data_header_.data_cmd_id_ = sp_referee::RADAR_ALLY_BUFF_CMD;
        cmd.robot_interaction_data_header_.sender_id_    = robot_info_.robot_id_;
        cmd.robot_interaction_data_header_.receiver_id_  = i;
        cmd.hero_heal_      = radar_ally_buff_cmd_ref_.hero_heal;
        cmd.hero_cool_      = radar_ally_buff_cmd_ref_.hero_cool;
        cmd.hero_def_       = radar_ally_buff_cmd_ref_.hero_def;
        cmd.hero_vuln_      = radar_ally_buff_cmd_ref_.hero_vuln;
        cmd.hero_atk_       = radar_ally_buff_cmd_ref_.hero_atk;
        cmd.engineer_heal_  = radar_ally_buff_cmd_ref_.engineer_heal;
        cmd.engineer_cool_  = radar_ally_buff_cmd_ref_.engineer_cool;
        cmd.engineer_def_   = radar_ally_buff_cmd_ref_.engineer_def;
        cmd.engineer_vuln_  = radar_ally_buff_cmd_ref_.engineer_vuln;
        cmd.engineer_atk_   = radar_ally_buff_cmd_ref_.engineer_atk;
        cmd.infantry3_heal_ = radar_ally_buff_cmd_ref_.infantry3_heal;
        cmd.infantry3_cool_ = radar_ally_buff_cmd_ref_.infantry3_cool;
        cmd.infantry3_def_  = radar_ally_buff_cmd_ref_.infantry3_def;
        cmd.infantry3_vuln_ = radar_ally_buff_cmd_ref_.infantry3_vuln;
        cmd.infantry3_atk_  = radar_ally_buff_cmd_ref_.infantry3_atk;
        cmd.infantry4_heal_ = radar_ally_buff_cmd_ref_.infantry4_heal;
        cmd.infantry4_cool_ = radar_ally_buff_cmd_ref_.infantry4_cool;
        cmd.infantry4_def_  = radar_ally_buff_cmd_ref_.infantry4_def;
        cmd.infantry4_vuln_ = radar_ally_buff_cmd_ref_.infantry4_vuln;
        cmd.infantry4_atk_  = radar_ally_buff_cmd_ref_.infantry4_atk;
        cmd.sentry_heal_    = radar_ally_buff_cmd_ref_.sentry_heal;
        cmd.sentry_cool_    = radar_ally_buff_cmd_ref_.sentry_cool;
        cmd.sentry_def_     = radar_ally_buff_cmd_ref_.sentry_def;
        cmd.sentry_vuln_    = radar_ally_buff_cmd_ref_.sentry_vuln;
        cmd.sentry_atk_     = radar_ally_buff_cmd_ref_.sentry_atk;
        cmd.sentry_posture_ = radar_ally_buff_cmd_ref_.sentry_posture;
        cmd.hero_status_      = radar_ally_buff_cmd_ref_.hero_status;
        cmd.engineer_status_  = radar_ally_buff_cmd_ref_.engineer_status;
        cmd.infantry3_status_ = radar_ally_buff_cmd_ref_.infantry3_status;
        cmd.infantry4_status_ = radar_ally_buff_cmd_ref_.infantry4_status;
        cmd.sentry_status_    = radar_ally_buff_cmd_ref_.sentry_status;
        pack(reinterpret_cast<uint8_t *>(&tx_buffer_),
             reinterpret_cast<uint8_t *>(&cmd),
             sp_referee::ROBOT_INTERACTIVE_DATA_CMD, data_len);
    }
    break;
}
```
