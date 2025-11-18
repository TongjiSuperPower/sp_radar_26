import cv2
import  numpy as  np

if __name__ == "__main__":
    cap = cv2.VideoCapture("./src/main/detection/media/simulate_dart.mp4")

    first_frame = True
    last_thresh = None

    flashing_count = 0

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # 转换为灰度图像
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 二值化
            ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

            if first_frame:
                first_frame = False
                last_thresh = thresh
                continue
            else:
                # 计算帧间差异
                diff_sum = 0
                rows, cols = thresh.shape  # 灰度图只有2个维度
                
                # 使用NumPy计算差异更高效
                diff = cv2.absdiff(thresh, last_thresh)

                # 定义内核（核大小根据噪声大小调整）
                size = 100
                kernel = np.ones((size,size), np.uint8)

                # 开运算（先腐蚀后膨胀）- 去除小白点噪声
                opening = cv2.morphologyEx(diff, cv2.MORPH_OPEN, kernel, iterations=1)

                # 闭运算（先膨胀后腐蚀）- 去除小黑点噪声
                closing = cv2.morphologyEx(diff, cv2.MORPH_CLOSE, kernel, iterations=1)

                # 同时使用开闭运算
                clean = cv2.morphologyEx(diff, cv2.MORPH_CLOSE, kernel, iterations=1)
                
                diff_sum = diff.sum()
                avg_diff = diff_sum / (rows * cols)
                print(f"Average difference: {avg_diff}")

                if avg_diff > 1:
                    flashing_count += 1
                    print("flashing")

                last_thresh = thresh

            # 显示结果（可选）
            cv2.imshow('Binary', thresh)
            cv2.imshow('Difference', diff)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f"flashing count = {flashing_count}")
    