import cv2
import argparse

def extract_video_segment(input_path, output_path, start_time, end_time):
    """
    截取视频中的一段并保存为新的视频文件
    
    参数:
        input_path: 输入视频路径
        output_path: 输出视频路径
        start_time: 开始时间(秒)
        end_time: 结束时间(秒)
    """
    # 打开视频文件
    cap = cv2.VideoCapture(input_path)
    if not cap.isOpened():
        print(f"无法打开视频文件: {input_path}")
        return
    
    # 获取视频基本信息
    fps = cap.get(cv2.CAP_PROP_FPS)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    # 计算开始和结束的帧数
    start_frame = int(start_time * fps)
    end_frame = int(end_time * fps)
    
    # 确保结束帧不超过视频总帧数
    end_frame = min(end_frame, total_frames - 1)
    
    # 设置视频编码器 (根据系统可能需要调整)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 或者 'XVID' 对于avi
    
    # 创建视频写入对象
    out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    
    # 跳转到开始帧
    cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    
    # 读取并写入指定范围内的帧
    current_frame = start_frame
    while current_frame <= end_frame:
        ret, frame = cap.read()
        if not ret:
            break
        
        out.write(frame)
        current_frame += 1
        print(f"Has save {current_frame} frame.")
    
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
    print(f"视频片段已保存到: {output_path}")

if __name__ == "__main__":
    # 设置命令行参数
    # parser = argparse.ArgumentParser(description='截取视频片段').
    # parser.add_argument('input', help='输入视频文件路径')
    # parser.add_argument('output', help='输出视频文件路径')
    # parser.add_argument('start', type=float, help='开始时间(秒)')
    # parser.add_argument('end', type=float, help='结束时间(秒)')
    
    # args = parser.parse_args()
    
    input = "/media/squirtle/KESU/records/20240516BO302DQ.mp4"
    output = "./src/main/detection/media/dart.mp4"
    start  = 368
    end = 380

    # 调用函数截取视频
    extract_video_segment(input, output, start, end)