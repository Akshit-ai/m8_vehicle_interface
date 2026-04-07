// include basic, required headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
// ROS2
#include "rclcpp/rclcpp.hpp"

// include all of the autoware header (hpp) includes required for the vehicle interface
#include <autoware_control_msgs/msg/control.hpp>
#include <tier4_vehicle_msgs/msg/vehicle_emergency_stamped.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
// ...
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>
#include <tier4_vehicle_msgs/msg/actuation_command.hpp>
// ...

// then include all of your own required headers (message definitions)
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
//#include <nav_msgs/msg/Twist.hpp>
#include <zgw_interfaces/msg/vehicle_control_data.hpp>
#include <zgw_interfaces/msg/vehicle_data.hpp>
#include <zgw_interfaces/srv/vehicle_set_indicator.hpp>
#include <zgw_interfaces/srv/vehicle_set_led_steering_wheel.hpp>

using std::placeholders::_1;

// actual class
class M8_Vehicle_Interface : public rclcpp::Node
{
public:
    M8_Vehicle_Interface();
private:
    // From Autoware to the vehicle (control the car)
    // Autoware Subscribers:
    rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
    rclcpp::Subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>::SharedPtr vehicle_emercency_sub_;
    rclcpp::Subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>::SharedPtr turn_indicators_sub_;
    // ...
    // Position Subscribergeometry msgs twistwithcovariance
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
    // Vehicle Control Data Publisher M8:
    rclcpp::Publisher<zgw_interfaces::msg::VehicleControlData>::SharedPtr vehicle_control_data_pub_;
    // Vehicle Control Data Publisher CARLA:
    rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationCommandStamped>::SharedPtr carla_actuation_pub_;
    // ?

    // From vehicle to Autoware
    // Subscriber: Vehicle Data!s
    rclcpp::Subscription<zgw_interfaces::msg::VehicleData>::SharedPtr vehicle_data_sub_;
    // Publishers (Autoware Topics!)
    rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr turn_indicators_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr hazard_lights_pub_;
    rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
    rclcpp::Publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>::SharedPtr actuation_status_pub_;   
    // ...
    
    // autoware command messages
    // autoware_control_msgs::msg::AckermannControlCommand::ConstSharedPtr control_cmd_ptr_;
    
    // callbacks
    void callback_control_cmd(const autoware_control_msgs::msg::Control::ConstSharedPtr msg);
    void callback_emergency_cmd(const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg);
    void callback_turn_indicators_cmd(const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg);
    void callback_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);
    // const autoware_control_msgs::msg::AckermannControlCommand::ConstSharedPtr msg);
    
    void VehicleDataSubscriptionCallback(const zgw_interfaces::msg::VehicleData::ConstSharedPtr msg);

    // others (helper functions: inside the other cpp file)
    void vehicle_set_turn_lights(uint8_t command);
    std::pair<double, double> calculate_torque_values(float velocity_desired);
    
    // Valiables
    rclcpp::Client<zgw_interfaces::srv::VehicleSetIndicator>::SharedPtr turn_lights_command_client;
    bool autoware_emergency = false;
    uint8_t prev_turnlights_command = 0;
    double engine_torque, brake_torque = 0.0, max_engine_torque = 32000,
        Kp = 1.150, Ki = 0.013, Kd = -0.0001, frequency = 100.0,
        v, st = 1/frequency, e_v, inte_v, deri_v, e_v_prev, acc;
};
