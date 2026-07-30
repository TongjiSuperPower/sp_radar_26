# 雷达 0x0301 多机通信子编码协议

> 发送方：雷达（红9 / 蓝109）  
> 通信方式：0x0301（机器人间通信）→ 自定义子编码  
> 数据均为小端序（Little Endian），`__packed` 紧凑排列

---

## ⚠️ 本次改动

**上期：** 合并原 0x0212（HP）、0x0213（弹药）、0x0214（场地）、0x0215（Buff）为单条 **0x0212**（75 字节）。
**本期：** 新增 **0x0213**，用于广播对方空中机器人被反制状态。

| 子编码 | 变更 |
|--------|------|
| 0x0210 | 不变 |
| 0x0211 | 不变 |
| 0x0212 | 不变（合并包），75 字节 |
| **0x0213** | 🆕 **新增**：对方空中机器人被反制状态，7 字节 |

**发送频率：**

| 子编码 | 频率 | 说明 |
|--------|------|------|
| 0x0210 | 事件触发 | 敌方飞镖闸门开启时发送（无固定周期） |
| 0x0211 | 事件触发 | 新位置数据到达时发送（相机 ~5Hz，信息波 ~10Hz） |
| 0x0212 | **1 Hz（暂定）** | 定时发送，底层数据以 10Hz 刷新 |
| 0x0213 | **事件触发** | 0x020C 包到达时即时转发（1Hz） |

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
| 0x0211 | 位置信息 | 仅哨兵 | 含数据来源标志 |
| **0x0212** | **合并数据** | 全员（除 inf5） | **HP + 弹药 + 场地 + Buff，75 字节** |
| 0x0213 | 空中反制状态 | 全员（除 inf5） | 1=对方空中机器人被反制 |

---

## 0x0210 — 敌方飞镖警告（7 字节）

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

## 0x0212 — 合并数据（75 字节）

> 合并原 0x0212（HP）、0x0213（弹药）、0x0214（场地）、0x0215（Buff）为单一数据包。
> 接收方：全员（除 inf5）。发送频率：**1 Hz**。


| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 2 | uint16 | hero_hp | 英雄血量 |
| 8 | 2 | uint16 | engineer_hp | 工程血量 |
| 10 | 2 | uint16 | infantry3_hp | 三号步兵血量 |
| 12 | 2 | uint16 | infantry4_hp | 四号步兵血量 |
| 14 | 2 | uint16 | sentry_hp | 哨兵血量 |
| 16 | 2 | uint16 | hero_ammo | 英雄弹药 |
| 18 | 2 | uint16 | infantry3_ammo | 三号步兵弹药 |
| 20 | 2 | uint16 | infantry4_ammo | 四号步兵弹药 |
| 22 | 2 | uint16 | aerial_ammo | 空中机器人弹药 |
| 24 | 2 | uint16 | sentry_ammo | 哨兵弹药 |
| 26 | 2 | uint16 | remain_coins | 剩余金币 |
| 28 | 2 | uint16 | total_coins | 总金币 |
| 30 | 4 | uint32 | status_flags | 场地状态标志位 |
| 34 | 1 | uint8 | hero_heal | 英雄回血 |
| 35 | 2 | uint16 | hero_cool | 英雄冷却 |
| 37 | 1 | uint8 | hero_def | 英雄防御 |
| 38 | 1 | uint8 | hero_vuln | 英雄易伤 |
| 39 | 2 | uint16 | hero_atk | 英雄攻击 |
| 41 | 1 | uint8 | engineer_heal | 工程回血 |
| 42 | 2 | uint16 | engineer_cool | 工程冷却 |
| 44 | 1 | uint8 | engineer_def | 工程防御 |
| 45 | 1 | uint8 | engineer_vuln | 工程易伤 |
| 46 | 2 | uint16 | engineer_atk | 工程攻击 |
| 48 | 1 | uint8 | infantry3_heal | 步兵3回血 |
| 49 | 2 | uint16 | infantry3_cool | 步兵3冷却 |
| 51 | 1 | uint8 | infantry3_def | 步兵3防御 |
| 52 | 1 | uint8 | infantry3_vuln | 步兵3易伤 |
| 53 | 2 | uint16 | infantry3_atk | 步兵3攻击 |
| 55 | 1 | uint8 | infantry4_heal | 步兵4回血 |
| 56 | 2 | uint16 | infantry4_cool | 步兵4冷却 |
| 58 | 1 | uint8 | infantry4_def | 步兵4防御 |
| 59 | 1 | uint8 | infantry4_vuln | 步兵4易伤 |
| 60 | 2 | uint16 | infantry4_atk | 步兵4攻击 |
| 62 | 1 | uint8 | sentry_heal | 哨兵回血 |
| 63 | 2 | uint16 | sentry_cool | 哨兵冷却 |
| 65 | 1 | uint8 | sentry_def | 哨兵防御 |
| 66 | 1 | uint8 | sentry_vuln | 哨兵易伤 |
| 67 | 2 | uint16 | sentry_atk | 哨兵攻击 |
| 69 | 1 | uint8 | sentry_posture | 哨兵姿态 |
| 70 | 1 | uint8 | hero_status | 英雄存活状态 |
| 71 | 1 | uint8 | engineer_status | 工程存活状态 |
| 72 | 1 | uint8 | infantry3_status | 步兵3存活状态 |
| 73 | 1 | uint8 | infantry4_status | 步兵4存活状态 |
| 74 | 1 | uint8 | sentry_status | 哨兵存活状态 |

---

## 0x0213 — 空中反制状态（7 字节）

> 接收方：全员（除 inf5）。数据来源：0x020C bit 13。
> 对方空中机器人被反制时值为 1，未被反制为 0。

| 偏移 | 长度 | 类型 | 字段 | 说明 |
|------|------|------|------|------|
| 0 | 6 | — | header | 公共帧头 |
| 6 | 1 | uint8 | aerial_countered | 1=对方空中机器人被反制 |

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