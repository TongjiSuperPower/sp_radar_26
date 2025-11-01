#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'Image',  # 话题名称
            self.image_callback,
            10)  # 队列大小
        self.subscription  # 防止未使用变量警告
        self.bridge = CvBridge()
        
        # 创建OpenCV窗口
        cv2.namedWindow("Image Viewer", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Image Viewer", 800, 600)
        
    def image_callback(self, msg):
        try:
            # 将ROS Image消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 显示图像
            cv2.imshow("Image Viewer", cv_image)
            cv2.waitKey(1)  # 必要的延迟以允许窗口更新
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def destroy_node(self):
        # 清理OpenCV窗口
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # 清理
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()