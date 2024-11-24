#include <memory>
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cstdint>

#define _USE_MATH_DEFINES

#include <iostream> // Including input-output stream header file
#include <math.h>   // Including math functions header file

using namespace std; // Using the standard namespace

float latitude;
float longitude;
float altitude;
sensor_msgs::msg::NavSatFix::SharedPtr data_gps;
geometry_msgs::msg::PoseArray::SharedPtr data_turbines;
sensor_msgs::msg::Imu::SharedPtr data_imu;

class Data : public rclcpp::Node
{

public:
    Data() : Node("gps_subscriber")
    {
        // Subscription for IMU 
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/aquabot/sensors/imu/imu/data", 10, std::bind(&Data::imu_callback, this, std::placeholders::_1));

        // Subscription for windturbines_positions
        sub_windturbines_positions = this->create_subscription<geometry_msgs::msg::PoseArray>("/aquabot/ais_sensor/windturbines_positions", 10, std::bind(&Data::windturbines_positions_callback, this, std::placeholders::_1));

        // Subscription for GPS boat 
        sub_gps = this->create_subscription<sensor_msgs::msg::NavSatFix>("/aquabot/sensors/gps/gps/fix", 10, std::bind(&Data::gps_callback, this, std::placeholders::_1));

        boat_data_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps_topic_mathys", 10);
        turbines_data_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("turbines_topic_mathys", 10);
        imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_topic_mathys", 10);

        timer_2 = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&Data::turbines_timer_callback, this));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&Data::timer_callback, this));
    }

private:
    int i = 0;
    void turbines_timer_callback()
    {
        if (start == true && i < 5)
        {
            geometry_msgs::msg::PoseArray msg = *data_turbines;
            turbines_data_publisher_->publish(msg);
            i++;
        }
    }

    void timer_callback()
    {
        if (start == true)
        {
            sensor_msgs::msg::NavSatFix msg = *data_gps;
            sensor_msgs::msg::Imu msg1 = *data_imu;
            geometry_msgs::msg::PoseArray msg2 = *data_turbines;
            boat_data_publisher_->publish(msg);
            imu_data_publisher_->publish(msg1);
            turbines_data_publisher_->publish(msg2);
        }
    }

    void windturbines_positions_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        data_turbines = msg;
        start = true;
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        data_gps = msg;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        data_imu = msg;
    }

    bool start = false;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_windturbines_positions;

    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr boat_data_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr turbines_data_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_2;
    rclcpp::TimerBase::SharedPtr timer_3;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Data>());
    rclcpp::shutdown();
    return 0;
}