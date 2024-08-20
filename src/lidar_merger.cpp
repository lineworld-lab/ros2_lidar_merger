#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarMerger : public rclcpp::Node
{
    public:
        LidarMerger()
        : Node("lidar_merger")
        {
            // Default values of the parameters are set for the LiDAR model "SLAMTEC LPX-T1"
            auto scan_data_length_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            scan_data_length_descriptor.description = "The length of scan data array that LiDAR returns";
            this->declare_parameter("scan_data_length", 6120, scan_data_length_descriptor);

            auto front_lidar_offset_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            front_lidar_offset_descriptor.description = "The offset of the front LiDAR in radians (positive is counter-clockwise)";
            this->declare_parameter("front_lidar_offset", 1.0/4.0 * M_PI, front_lidar_offset_descriptor);

            auto rear_lidar_offset_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            rear_lidar_offset_descriptor.description = "The offset of the rear LiDAR in radians (positive is counter-clockwise)";
            this->declare_parameter("rear_lidar_offset", 3.0/4.0 * M_PI, rear_lidar_offset_descriptor);

            auto lidar_installed_reversely_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            lidar_installed_reversely_descriptor.description = "Whether the rear LiDAR is installed in reverse";
            this->declare_parameter("lidar_installed_reversely", true, lidar_installed_reversely_descriptor);

            auto lidar_scan_data_direction_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            lidar_scan_data_direction_descriptor.description = "The direction of scan data array that LiDAR returns. True is clockwise, False is counter-clockwise";
            this->declare_parameter("lidar_scan_data_direction", true, lidar_scan_data_direction_descriptor);

            auto x_diff_from_rear_lidar_to_front_lidar_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            x_diff_from_rear_lidar_to_front_lidar_descriptor.description = "The distance between the rear LiDAR and the front LiDAR in the x-axis";
            this->declare_parameter("x_diff_from_rear_lidar_to_front_lidar", 0.49, x_diff_from_rear_lidar_to_front_lidar_descriptor);

            auto y_diff_from_rear_lidar_to_front_lidar_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
            y_diff_from_rear_lidar_to_front_lidar_descriptor.description = "The distance between the rear LiDAR and the front LiDAR in the y-axis";
            this->declare_parameter("y_diff_from_rear_lidar_to_front_lidar", -0.73, y_diff_from_rear_lidar_to_front_lidar_descriptor);

            int lidar_scan_data_direction = this->get_parameter("lidar_scan_data_direction").as_bool() ? 1 : -1;
            int lidar_installed_reversely = this->get_parameter("lidar_installed_reversely").as_bool()? -1 : 1;
            scan_data_direction_ = lidar_scan_data_direction * lidar_installed_reversely;

            front_lidar_offset_ = this->get_parameter("front_lidar_offset").as_double();
            rear_lidar_offset_ = this->get_parameter("rear_lidar_offset").as_double();
            x_diff_from_rear_lidar_to_front_lidar_ = this->get_parameter("x_diff_from_rear_lidar_to_front_lidar").as_double();
            y_diff_from_rear_lidar_to_front_lidar_ = this->get_parameter("y_diff_from_rear_lidar_to_front_lidar").as_double();

            scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
            front_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "front_scan", 10, std::bind(&LidarMerger::front_scan_callback, this, std::placeholders::_1));
            rear_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "rear_scan", 10, std::bind(&LidarMerger::rear_scan_callback, this, std::placeholders::_1)
            );
        }
    
    private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr front_scan_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr rear_scan_subscriber_;
        sensor_msgs::msg::LaserScan::SharedPtr front_scan_msg_;
        int scan_data_direction_;
        float front_lidar_offset_;
        float rear_lidar_offset_;
        float x_diff_from_rear_lidar_to_front_lidar_;
        float y_diff_from_rear_lidar_to_front_lidar_;

        void front_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr front_scan_msg)
        {
            front_scan_msg_ = front_scan_msg;
        }

        void rear_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr rear_scan_msg)
        {
            if (!front_scan_msg_) {
                return;
            }

            int array_length = static_cast<int>(front_scan_msg_->ranges.size());

            for (int front_object_index = 0; front_object_index < array_length; front_object_index++) {
                if (front_scan_msg_->ranges[front_object_index] == std::numeric_limits<float>::infinity()) {
                    continue;
                }

                float front_object_bearing_rad = (front_object_index / static_cast<float>(array_length)) * 2.0 * M_PI;
                float front_object_math_angle = M_PI / 2 + scan_data_direction_ * front_object_bearing_rad;

                float front_object_x_ = front_scan_msg_->ranges[front_object_index] * cos(front_object_math_angle + front_lidar_offset_);
                float front_object_y_ = front_scan_msg_->ranges[front_object_index] * sin(front_object_math_angle + front_lidar_offset_);

                float rear_object_x = front_object_x_ - x_diff_from_rear_lidar_to_front_lidar_;
                float rear_object_y = front_object_y_ - y_diff_from_rear_lidar_to_front_lidar_;

                float rear_object_math_angle = atan2(rear_object_y, rear_object_x);
                float rear_object_bearing_rad = M_PI / 2 - rear_object_math_angle;
                int rear_object_index = round((rear_object_bearing_rad + scan_data_direction_ * rear_lidar_offset_) / (2.0 * M_PI) * array_length);
                float rear_object_distance = sqrt(pow(rear_object_x, 2) + pow(rear_object_y, 2));

                if (rear_object_index < 0) {
                    rear_object_index += array_length;
                }

                if (rear_object_index >= array_length) {
                    rear_object_index -= array_length;
                }

                rear_scan_msg->ranges[rear_object_index] = rear_object_distance;
                rear_scan_msg->intensities[rear_object_index] = front_scan_msg_->intensities[front_object_index];

            }

            scan_publisher_->publish(*rear_scan_msg);
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMerger>());
    rclcpp::shutdown();

    return 0;
}