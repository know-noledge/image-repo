#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert the ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        if (!cv_ptr) {
            RCLCPP_WARN(this->get_logger(), "Could not convert image");
            return;
        }

        // Display the image
        //cv::imshow("Received Image", cv_ptr->image);
        //cv::waitKey(1);  // Wait for a short period to allow image display
        RCLCPP_INFO(this->get_logger(), "Image received");
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
