# 四个 float32 数值，表示边界框的四个坐标或大小
float32 x_min
float32 y_min
float32 x_max
float32 y_max

float32 class_confidence
int8 class_id

# 车体检测器自己的得分（不是装甲板得分）。用于区分"真车体但装甲板没认出来"和"压根不是车的误检"。
# 两个目标框重叠时也用它挑更可信的那个框，替代原来"取最后一个命中的框"的做法。
# < 0 表示该路径没有车体得分可用。
float32 car_confidence
