#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class CircleDetector : public rclcpp::Node
{
public:
    CircleDetector()
        : Node("circle_detector")
    {
        // Subscribe to an image topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/coppelia/camera/image", 10,
            std::bind(&CircleDetector::image_callback, this, std::placeholders::_1));

    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert image to grayscale
        cv::Mat gray;
        cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

        // Detect circles using Hough Circle Transform
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, gray.rows / 8, 100, 30, 0, 0);

        // Draw detected circles
        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);

            // Draw the circle center
            cv::circle(cv_ptr->image, center, 3, cv::Scalar(0, 255, 0), -1);
            // Draw the circle outline
            cv::circle(cv_ptr->image, center, radius, cv::Scalar(0, 0, 255), 2);
        }

        // Display the result
        cv::imshow("Detected Circles", cv_ptr->image);
        cv::waitKey(1); // Display for 1ms
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CircleDetector>();
    RCLCPP_INFO(node->get_logger(), "Circle detector node started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
