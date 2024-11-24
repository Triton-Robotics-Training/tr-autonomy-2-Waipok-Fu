#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class AngleCalculator : public rclcpp::Node
{
public:
    AngleCalculator() : Node("angle_calculator"), current_angle_(0.0)
    {
        // subscript /robotcam topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robotcam", 10, std::bind(&AngleCalculator::image_callback, this, std::placeholders::_1));

        //subscript /current_angle topic
        angle_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/current_angle", 10, std::bind(&AngleCalculator::current_angle_callback, this, std::placeholders::_1));

        //create/desired_angle publisher
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AngleCalculator::process_and_publish_angle, this));
    }

private:
    // image subcript callback
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // convert ROS image to CV image
            cv_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge exception: %s", e.what());
        }
    }

    // current angle subscript callback
    void current_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_angle_ = msg->data; // update current angle
    }

    // process and publish target angle
    void process_and_publish_angle()
    {
        if (cv_image_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No image data received yet.");
            return;
        }

        // extract target center
        cv::Point target_center = find_red_region_center(cv_image_);
        if (target_center.x == -1)
        {
            RCLCPP_WARN(this->get_logger(), "No red region detected.");
            return;
        }

        // compute target angle
        float target_angle = calculate_target_angle(target_center, cv_image_.cols);

        // compute target relative angle
        float relative_angle = current_angle_ + target_angle;

        // publish target angle
        auto msg = std_msgs::msg::Float32();
        msg.data = relative_angle;
        angle_publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published relative angle: %f", relative_angle);
    }

    // extract target center
    cv::Point find_red_region_center(const cv::Mat &image)
    {
        cv::Mat hsv_image, mask, mask_upper;
        // convert to hsv
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

        // extract red region
        cv::inRange(hsv_image, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask); // lower limit
        cv::inRange(hsv_image, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask_upper); // upper limit
        mask = mask | mask_upper;

        // get contours
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (contours.empty())
            return cv::Point(-1, -1); // fail to detect red region

        // get the center of the biggest region
        double max_area = 0;
        cv::Point center(-1, -1);
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area > max_area)
            {
                max_area = area;
                cv::Moments m = cv::moments(contour);
                center = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
            }
        }

        return center;
    }

    // 计算目标偏移角度
    float calculate_target_angle(const cv::Point &target_center, int image_width)
    {
        // 相机视场角 (FOV) 为 90 度，屏幕左右各占 45 度
        float fov = 90.0;
        float pixels_per_degree = image_width / fov;

        // 偏移像素转换为角度
        float pixel_offset = target_center.x - (image_width / 2);
        float angle_offset = pixel_offset / pixels_per_degree;

        // 转换为弧度
        return -angle_offset * (M_PI / 180.0);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    cv::Mat cv_image_;
    float current_angle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngleCalculator>());
    rclcpp::shutdown();
    return 0;
}

