# 合并0x0A02~0x0A05的接收，以1Hz发给sp_referee的统一话题
# 0x0A02: HP数据
uint16 hero_hp
uint16 engineer_hp
uint16 infantry3_hp
uint16 infantry4_hp
uint16 sentry_hp

# 0x0A03: 弹药数据
uint16 hero_ammo
uint16 infantry3_ammo
uint16 infantry4_ammo
uint16 aerial_ammo
uint16 sentry_ammo

# 0x0A04: 场地状态数据
uint16 remain_coins
uint16 total_coins
uint32 status_flags

# 0x0A05: Buff数据
uint8 hero_heal
uint16 hero_cool
uint8 hero_def
uint8 hero_vuln
uint16 hero_atk
uint8 engineer_heal
uint16 engineer_cool
uint8 engineer_def
uint8 engineer_vuln
uint16 engineer_atk
uint8 infantry3_heal
uint16 infantry3_cool
uint8 infantry3_def
uint8 infantry3_vuln
uint16 infantry3_atk
uint8 infantry4_heal
uint16 infantry4_cool
uint8 infantry4_def
uint8 infantry4_vuln
uint16 infantry4_atk
uint8 sentry_heal
uint16 sentry_cool
uint8 sentry_def
uint8 sentry_vuln
uint16 sentry_atk
uint8 sentry_posture
uint8 hero_status
uint8 engineer_status
uint8 infantry3_status
uint8 infantry4_status
uint8 sentry_status
