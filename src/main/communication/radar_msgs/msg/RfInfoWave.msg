std_msgs/Header header

# ---- 0x0A01: 机器人坐标 (cm), 6台 × (x, y), int16 ----
# 顺序: hero, eng, inf3, inf4, aerial, sentry
int16[12] positions

# ---- 0x0A02: 机器人血量, uint16 ----
# 顺序: hero, eng, inf3, inf4, sentry (跳过保留位)
uint16[5] hp

# ---- 0x0A03: 剩余弹量, uint16 ----
# 顺序: hero, inf3, inf4, aerial, sentry
uint16[5] ammo

# ---- 0x0A04: 经济/场地状态 ----
uint16 remaining_coins
uint16 total_coins
uint32 status_flags

# ---- 0x0A05: Buff (5台 × 5种buff) 和 哨兵姿态 ----
# 每台机器人: heal, cool, def, vuln, atk (均为uint8)
# 顺序: hero, eng, inf3, inf4, sentry
uint8[25] buffs
uint8 sentry_posture