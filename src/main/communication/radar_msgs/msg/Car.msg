int8 class_id
float32 x
float32 y

# 装甲板分类得分（来自 Bbox.class_confidence）。
#   > 0  分类成功的得分，越高越可信
#   = 0  跑了装甲板检测但没有超过阈值的类别
#   < 0  没有装甲板结果（车体框存在但装甲板流程没出结果，或该点压根没有对应的车体框）
# 用途：class_id 是否会写进轨迹的身份历史，由这个值把关，避免低分误分类抢走真 id。
float32 class_confidence

# 车体检测器得分（来自 Bbox.car_confidence）。< 0 表示这个点没有对应的车体框。
# 和 class_confidence 一起决定这个观测有没有资格断言一个身份。
float32 car_confidence
