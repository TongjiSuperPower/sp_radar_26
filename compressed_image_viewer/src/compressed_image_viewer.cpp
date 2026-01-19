#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include <string>
#include <chrono>

class CompressedImageViewer : public rclcpp::Node
{
public:
    CompressedImageViewer() : Node("compressed_image_viewer")
    {
        // 参数：订阅的话题名称
        this->declare_parameter<std::string>("topic", "camera/image/compressed");
        this->declare_parameter<bool>("show_image", true);
        this->declare_parameter<bool>("resize_image", false);
        this->declare_parameter<int>("resize_width", 640);
        this->declare_parameter<int>("resize_height", 480);
        
        std::string topic = this->get_parameter("topic").as_string();
        show_image_ = this->get_parameter("show_image").as_bool();
        resize_image_ = this->get_parameter("resize_image").as_bool();
        resize_width_ = this->get_parameter("resize_width").as_int();
        resize_height_ = this->get_parameter("resize_height").as_int();
        
        // 输出参数信息
        RCLCPP_INFO(this->get_logger(), "订阅话题: %s", topic.c_str());
        RCLCPP_INFO(this->get_logger(), "显示图像: %s", show_image_ ? "是" : "否");
        
        if (show_image_) {
            cv::namedWindow("Compressed Image Viewer", cv::WINDOW_AUTOSIZE);
        }
        
        // 订阅压缩图像话题
        compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            topic,
            rclcpp::SensorDataQoS(),
            std::bind(&CompressedImageViewer::compressedImageCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "压缩图像查看器已启动");
        RCLCPP_INFO(this->get_logger(), "按 'q' 或 ESC 退出");
        RCLCPP_INFO(this->get_logger(), "按 's' 保存当前图像");
        
        // 如果显示图像，创建定时器检查窗口状态
        if (show_image_) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CompressedImageViewer::checkWindow, this));
        }
    }
    
    ~CompressedImageViewer()
    {
        if (show_image_) {
            cv::destroyWindow("Compressed Image Viewer");
        }
    }

private:
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try {
            // 将压缩图像转换为OpenCV格式
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            
            if (image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "无法解码图像");
                return;
            }
            
            // 更新帧率统计
            frame_count_++;
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time_).count();
            
            if (elapsed >= 2) {
                double fps = frame_count_ / elapsed;
                RCLCPP_INFO(this->get_logger(), "接收帧率: %.2f FPS", fps);
                frame_count_ = 0;
                last_stat_time_ = now;
            }
            
            // 如果需要调整大小
            if (resize_image_ && !image.empty()) {
                cv::resize(image, image, cv::Size(resize_width_, resize_height_));
            }
            
            // 显示图像
            if (show_image_ && !image.empty()) {
                // 在图像上添加信息
                cv::Mat display_image = image.clone();
                
                // 添加话题名称
                std::string info = "Topic: " + this->get_parameter("topic").as_string();
                cv::putText(display_image, info, cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
                
                // 添加时间戳
                std::string timestamp = "Stamp: " + std::to_string(msg->header.stamp.sec) + 
                                       "." + std::to_string(msg->header.stamp.nanosec);
                cv::putText(display_image, timestamp, cv::Point(10, 60), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // 添加图像尺寸信息
                std::string size_info = "Size: " + std::to_string(image.cols) + 
                                       "x" + std::to_string(image.rows);
                cv::putText(display_image, size_info, cv::Point(10, 90), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                
                // 显示图像
                cv::imshow("Compressed Image Viewer", display_image);
                
                // 处理键盘输入
                int key = cv::waitKey(1);
                if (key == 'q' || key == 27) {  // 'q' 或 ESC
                    RCLCPP_INFO(this->get_logger(), "收到退出信号");
                    rclcpp::shutdown();
                } else if (key == 's') {  // 's' 保存图像
                    saveImage(image);
                } else if (key == 'f') {  // 'f' 切换全屏
                    toggleFullscreen();
                }
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge异常: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "异常: %s", e.what());
        }
    }
    
    void saveImage(const cv::Mat& image)
    {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        std::string filename = "saved_image_" + std::to_string(timestamp) + ".jpg";
        cv::imwrite(filename, image);
        RCLCPP_INFO(this->get_logger(), "图像已保存为: %s", filename.c_str());
    }
    
    void toggleFullscreen()
    {
        fullscreen_ = !fullscreen_;
        if (fullscreen_) {
            cv::setWindowProperty("Compressed Image Viewer", 
                                 cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        } else {
            cv::setWindowProperty("Compressed Image Viewer", 
                                 cv::WND_PROP_FULLSCREEN, cv::WINDOW_NORMAL);
        }
    }
    
    void checkWindow()
    {
        // 检查窗口是否被关闭
        if (cv::getWindowProperty("Compressed Image Viewer", cv::WND_PROP_VISIBLE) < 1) {
            RCLCPP_INFO(this->get_logger(), "窗口已关闭，退出程序");
            rclcpp::shutdown();
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    bool show_image_;
    bool resize_image_;
    bool fullscreen_ = false;
    int resize_width_;
    int resize_height_;
    
    int frame_count_ = 0;
    std::chrono::steady_clock::time_point last_stat_time_ = std::chrono::steady_clock::now();
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CompressedImageViewer>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    
    rclcpp::shutdown();
    cv::destroyAllWindows();
    
    return 0;
}