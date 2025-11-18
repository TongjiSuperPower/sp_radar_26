# 用于从视频中提取图片，并生成 filelist.txt 文件，用于校准
import cv2
import os
import random

# 输入视频路径和图片输出目录
input_video = "/home/lp1/RM/radar/tensorRT/code/yolov5_tensorrt/part1/code/tensorrt_cpp_v2.0/tensorrt_cpp/media/output_clip.mp4"  # 替换为您的视频路径
output_dir = "/home/lp1/RM/radar/tensorRT/code/yolov5_tensorrt/part1/code/tensorrt_cpp_v2.0/tensorrt_cpp/img"  # 存放图片的文件夹

# 创建输出文件夹（如果不存在）
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 打开视频文件
cap = cv2.VideoCapture(input_video)
frame_count = 0  # 用于计数帧

# 遍历视频每隔10帧保存一张图片
saved_images = []
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 每隔10帧保存一张图片
    if frame_count % 10 == 0:
        image_name = f"frame_{frame_count}.jpg"
        image_path = os.path.join(output_dir, image_name)
        cv2.imwrite(image_path, frame)
        saved_images.append(image_name)

    frame_count += 1

# 释放视频资源
cap.release()

# 随机选取200张图片的名称
selected_images = random.sample(saved_images, min(200, len(saved_images)))

# 将选中的图片名称保存到filelist.txt
with open("filelist.txt", "w") as f:
    for image_name in selected_images:
        f.write(image_name + "\n")

print("图片已成功保存，并生成 filelist.txt 文件。")
