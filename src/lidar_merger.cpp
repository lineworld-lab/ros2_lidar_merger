#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarMerger : public rclcpp::Node
{
    public:
        LidarMerger()
        : Node("lidar_merger")
        {
            auto first_lidar_yaw_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            first_lidar_yaw_descriptor.description = "The yaw angle of the first LiDAR";
            this->declare_parameter("first_lidar_yaw", rclcpp::PARAMETER_DOUBLE, first_lidar_yaw_descriptor);

            auto second_lidar_yaw_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            second_lidar_yaw_descriptor.description = "The yaw angle of the second LiDAR";
            this->declare_parameter("second_lidar_yaw", rclcpp::PARAMETER_DOUBLE, second_lidar_yaw_descriptor);

            auto x_diff_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            x_diff_descriptor.description = "The distance between the second LiDAR and the first LiDAR in the x-axis in relative coordinates";
            this->declare_parameter("x_diff", rclcpp::PARAMETER_DOUBLE, x_diff_descriptor);

            auto y_diff_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            y_diff_descriptor.description = "The distance between the second LiDAR and the first LiDAR in the y-axis in relative coordinates";
            this->declare_parameter("y_diff", rclcpp::PARAMETER_DOUBLE, y_diff_descriptor);

            f_yaw_ = this->get_parameter("first_lidar_yaw").as_double();
            s_yaw_ = this->get_parameter("second_lidar_yaw").as_double();
            x_diff_ = this->get_parameter("x_diff").as_double();
            y_diff_ = this->get_parameter("y_diff").as_double();

            scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
            first_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "first_scan", 10, std::bind(&LidarMerger::first_scan_callback, this, std::placeholders::_1));
            second_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "second_scan", 10, std::bind(&LidarMerger::second_scan_callback, this, std::placeholders::_1)
            );
        }

    
    private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr first_scan_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr second_scan_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr first_scan_msg_;
        float f_yaw_;
        float s_yaw_;
        float x_diff_;
        float y_diff_;

        void first_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr first_scan_msg)
        {
            first_scan_msg_ = first_scan_msg;
        }

        void second_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr second_scan_msg)
        {
            if (!first_scan_msg_) {
                return;
            }

            float f_min = first_scan_msg_->angle_min;
            float f_max = first_scan_msg_->angle_max;
            float f_length = static_cast<int>(second_scan_msg->ranges.size());

            float s_min = second_scan_msg->angle_min;
            float s_max = second_scan_msg->angle_max;
            float s_length = static_cast<int>(second_scan_msg->ranges.size());

            int s_new_length = s_length + s_length / std::abs(s_max - s_min) * (2.0 * M_PI - std::abs(s_max - s_min));

            std::vector<float> s_new_ranges(s_new_length, std::numeric_limits<float>::infinity());
            for (int i = 0; i < s_length; i++) {
                s_new_ranges[i] = second_scan_msg->ranges[i];
            }
    
            std::vector<float> s_new_intensities(s_new_length, 0.0);
            for (int i = 0; i < s_length; i++) {
                s_new_intensities[i] = second_scan_msg->intensities[i];
            }

            for (int f_index = 0; f_index < f_length; f_index++) {
                if (std::isinf(first_scan_msg_->ranges[f_index])) {
                    continue;
                }

                float f_angle;
                if (f_max > f_min) {
                    f_angle = M_PI / 2.0 + f_yaw_ + f_min + (f_max - f_min) * f_index / f_length;
                } else {
                    f_angle = M_PI / 2.0 + f_yaw_ + f_max + (f_min - f_max) * (f_length - f_index) / f_length;
                }

                float f_range = first_scan_msg_->ranges[f_index];
                float s_x = f_range * std::cos(f_angle) - x_diff_;
                float s_y = f_range * std::sin(f_angle) - y_diff_;

                float s_angle = std::atan2(s_y, s_x);

                int s_index;
                if (s_max > s_min) {
                    s_index = round((s_angle - (M_PI / 2.0 + s_yaw_ + s_min)) / (2.0 * M_PI) * s_new_length);
                } else {
                    s_index = round(s_new_length - ((M_PI / 2.0 + s_yaw_ + s_min + s_angle) / (2.0 * M_PI) * s_new_length));
                }
                float s_range = std::sqrt(s_x * s_x + s_y * s_y);

                if (s_index > s_new_length) {
                    s_index -= s_new_length;
                } else if (s_index < 0) {
                    s_index += s_new_length;
                }

                s_new_ranges[s_index] = s_range;
                s_new_intensities[s_index] = first_scan_msg_->intensities[f_index];
            }

            second_scan_msg->ranges = s_new_ranges;
            second_scan_msg->intensities = s_new_intensities;

            scan_publisher_->publish(*second_scan_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMerger>());
    rclcpp::shutdown();

    return 0;
}