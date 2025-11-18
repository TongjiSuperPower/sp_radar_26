#设置路径、大小、帧率、截取区间
import cv2
import numpy as np

# 输入视频路径
input_video = "/media/squirtle/KESU/records/20240516BO302DQ.mp4"  # 替换为您的视频路径
output_video = "./src/main/detection/media/output/dart.mp4"  # 输出视频路径

# 设置目标宽度和高度
target_width = 1280
target_height = 1024

# 设置输出帧率
output_fps = 60
# 设置截取时间（单位：秒）
start_time = 365
end_time =  375

# 打开视频文件
cap = cv2.VideoCapture(input_video)

# 获取视频的基本信息
input_fps = int(cap.get(cv2.CAP_PROP_FPS))
input_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
input_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 视频编码格式

# 创建视频写入对象
out = cv2.VideoWriter(output_video, fourcc, input_fps, (input_height, input_height))
# 计算开始和结束的帧数
start_frame = start_time * input_fps
end_frame = end_time * input_fps
cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

# 计算缩放比例
scale_w = target_width / input_width
scale_h = target_height / input_height
scale = min(scale_w, scale_h)

# 计算缩放后的尺寸
new_width = int(input_width * scale)
new_height = int(input_height * scale)

# 计算填充的灰色区域
pad_w = (target_width - new_width) // 2
pad_h = (target_height - new_height) // 2

current_frame = start_frame
while cap.isOpened() and current_frame < end_frame:
    ret, frame = cap.read()
    if not ret:
        break

    # # 缩放帧
    # resized_frame = cv2.resize(frame, (new_width, new_height))

    # # 创建灰色背景
    # gray_background = np.full((target_height, target_width, 3), 128, dtype=np.uint8)

    # # 将缩放后的帧放置在灰色背景上
    # gray_background[pad_h:pad_h + new_height, pad_w:pad_w + new_width] = resized_frame

    # 写入帧
    # out.write(gray_background)

    out.write(frame)
    current_frame += 1
    print("current_frame:", current_frame)


# 释放视频资源
cap.release()
out.release()

print("video saved to:", output_video)