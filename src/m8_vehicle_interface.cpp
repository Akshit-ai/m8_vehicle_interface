#include <m8_vehicle_interface.hpp>

// Constructor -> Set up all Publishers and Subscribers here!
M8_Vehicle_Interface::M8_Vehicle_Interface() : Node("M8_Vehicle_Interface") {
    std::cout << "Starting the Node" << std::endl;

    // Autoware to control the car
    // Subscriber declarations for the Autoware Control Topics:
    control_cmd_sub_ = create_subscription<autoware_control_msgs::msg::Control>(
        "/control/command/control_cmd", 10, std::bind(&M8_Vehicle_Interface::callback_control_cmd, this, _1));
    vehicle_emercency_sub_ = create_subscription<tier4_vehicle_msgs::msg::VehicleEmergencyStamped>(
        "/control/command/emergency_cmd", 10, std::bind(&M8_Vehicle_Interface::callback_emergency_cmd, this, _1));
    turn_indicators_sub_ = create_subscription<autoware_vehicle_msgs::msg::TurnIndicatorsCommand>(
        "/control/command/turn_indicators_cmd", 10, std::bind(&M8_Vehicle_Interface::callback_turn_indicators_cmd, this, _1));
    // ...
    // Subscriber for current position
    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>("/localization/twist_estimator/twist_with_covariance",  // geometry_msgs::msg::TwistWithCovarianceStamped // twist -> linear -> x
        10, std::bind(&M8_Vehicle_Interface::callback_twist, this, std::placeholders::_1));
    // Publisher for vehicle control data
    vehicle_control_data_pub_ = this->create_publisher<zgw_interfaces::msg::VehicleControlData>("vehicle_control_data", 10);
    carla_actuation_pub_ = create_publisher<tier4_vehicle_msgs::msg::ActuationCommandStamped>(
    "/control/command/actuation_cmd", rclcpp::QoS{1});
    // Vehicle Data to Autoware
    // Subscriber to Vehicle Data
    vehicle_data_sub_ = this->create_subscription<zgw_interfaces::msg::VehicleData>("m8_vehicle_data", // TODO: set correct name
        1, std::bind(&M8_Vehicle_Interface::VehicleDataSubscriptionCallback, this, std::placeholders::_1));
    // Publishers of vehicle data to Autoware
    gear_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(
        "/vehicle/status/gear_status", rclcpp::QoS{1});
    turn_indicators_pub_ = create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(
        "/vehicle/status/TurnIndicatorReport", rclcpp::QoS{1});
    velocity_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS{1});
    steering_status_pub_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS{1});
    hazard_lights_pub_ = create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(
        "/vehicle/status/hazard_lights_status", rclcpp::QoS{1});
    control_mode_pub_ = create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS{1});
    actuation_status_pub_ = create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
        "/vehicle/status/actuation_status", rclcpp::QoS{1});
    



    // Others:
    turn_lights_command_client = this->create_client<zgw_interfaces::srv::VehicleSetIndicator>("vehicle_set_indicator");
}

void M8_Vehicle_Interface::callback_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
    v = msg->twist.twist.linear.x;
}

// AUTOWARE to control the car
// Autoware Subscriber Callbacks here:

void M8_Vehicle_Interface::callback_control_cmd(
    const autoware_control_msgs::msg::Control::ConstSharedPtr msg) {
    float velocity_desired = msg->longitudinal.velocity;
    float steering_tire_angle = msg->lateral.steering_tire_angle;
    //velocity_desired = steering_tire_angle;

    auto control_msg = zgw_interfaces::msg::VehicleControlData();
  
    // Longitudinal Control: PID
    std::pair<double, double> carla_throttle_brake_pair; // Carla throttle value (0..1) will be found in carla_throttle_brake_pair.first | brake value between 0..1 in .second
    carla_throttle_brake_pair = calculate_torque_values(velocity_desired);
    // set torque-values calculated in the helper function:
    control_msg.engine_torque = engine_torque;
    control_msg.brake_torque = brake_torque;

    // Lateral Control: Conversion to curvature (TODO!)
    // Convert here to Curvature!

    // Lateral Control: steering angle to curvature conversion
    // BMW M8 Competition Coupe wheelbase = 2.827 meters
    const double wheelbase = 2.827;
    control_msg.curvature = std::tan(steering_tire_angle) / wheelbase;
  
    // Publish Topics:
    // Carla (0..1) control topics -> create new message definition for Carla!
    // CARLA Actuation Command Publisher
    auto carla_cmd = tier4_vehicle_msgs::msg::ActuationCommandStamped();
    carla_cmd.header.stamp = this->now();
    carla_cmd.actuation.accel_cmd = carla_throttle_brake_pair.first;
    carla_cmd.actuation.brake_cmd = carla_throttle_brake_pair.second;
    carla_cmd.actuation.steer_cmd = steering_tire_angle / 0.52;
    carla_actuation_pub_->publish(carla_cmd);

    // ... 
    
    // ZGW control topic
    // 
    control_msg.brake_torque_enable = control_msg.engine_torque_enable = control_msg.curvature_enable = true;
    vehicle_control_data_pub_->publish(control_msg);
}

void M8_Vehicle_Interface::callback_emergency_cmd(
    const tier4_vehicle_msgs::msg::VehicleEmergencyStamped::ConstSharedPtr msg) {
    if(msg->emergency != autoware_emergency) {
        autoware_emergency = msg->emergency;
        RCLCPP_INFO(this->get_logger(), "Emergency Mode Set to: %s", msg->emergency?"True":"False");
    }
}

void M8_Vehicle_Interface::callback_turn_indicators_cmd(
    const autoware_vehicle_msgs::msg::TurnIndicatorsCommand::ConstSharedPtr msg) {
    if(msg->command) vehicle_set_turn_lights(msg->command-1); // Change Turnlights only on request (0 = no request)
}

// ...

// Here is the callback for the way back (from the vehicle to Autoware):
void M8_Vehicle_Interface::VehicleDataSubscriptionCallback(
    const zgw_interfaces::msg::VehicleData::ConstSharedPtr msg) {
    // extract all necessary data here
    // ...
    uint8_t indicator_status = msg->indicator_status; // 0=off, 1=left, 2=right, 3=both
    // ...

    // ...

    // Turn Indicators Report Publisher
    auto turn_indicator_report_msg = autoware_vehicle_msgs::msg::TurnIndicatorsReport();
    if(0 == indicator_status) { // No blinking
        turn_indicator_report_msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE;
    } else if (1 == indicator_status) { // Left blinking
        turn_indicator_report_msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_LEFT;
    } else if (2 == indicator_status) {  // Right blinking
        turn_indicator_report_msg.report = autoware_vehicle_msgs::msg::TurnIndicatorsReport::ENABLE_RIGHT;
    }
    turn_indicator_report_msg.stamp = this->now();
    // Publish:
    turn_indicators_pub_->publish(turn_indicator_report_msg);
    
    // Velocity Report Publisher
    auto velocity_report_msg = autoware_vehicle_msgs::msg::VelocityReport();
    velocity_report_msg.header.stamp = this->now();
    velocity_report_msg.longitudinal_velocity = static_cast<float>(msg->velocity_long);
    velocity_report_msg.lateral_velocity = static_cast<float>(msg->velocity_cog);
    velocity_report_msg.heading_rate = static_cast<float>(msg->yaw_rate);
    velocity_status_pub_->publish(velocity_report_msg);

    // Steering Report Publisher
    auto steering_report_msg = autoware_vehicle_msgs::msg::SteeringReport();
    steering_report_msg.stamp = this->now();
    steering_report_msg.steering_tire_angle = static_cast<float>(msg->front_wheels_angle);
    steering_status_pub_->publish(steering_report_msg);

    // Hazard Lights Report Publisher
    auto hazard_lights_report_msg = autoware_vehicle_msgs::msg::HazardLightsReport();
    hazard_lights_report_msg.stamp = this->now();
    if(indicator_status == 3) { // 3 = both indicators = hazard lights
        hazard_lights_report_msg.report = autoware_vehicle_msgs::msg::HazardLightsReport::ENABLE;
    } else {
        hazard_lights_report_msg.report = autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE;
    }
    hazard_lights_pub_->publish(hazard_lights_report_msg);

    // Gear Report Publisher
    auto gear_report_msg = autoware_vehicle_msgs::msg::GearReport();
    gear_report_msg.stamp = this->now();
    if(msg->transmission_status == 0) {        // Park
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::PARK;
    } else if(msg->transmission_status == 1) { // Reverse
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::REVERSE;
    } else if(msg->transmission_status == 2) { // Neutral
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::NEUTRAL;
    } else if(msg->transmission_status == 3) { // Drive
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::DRIVE;
    } else {
    gear_report_msg.report = autoware_vehicle_msgs::msg::GearReport::NONE;
    }
    gear_status_pub_->publish(gear_report_msg);

    // Control Mode Report Publisher
    auto control_mode_report_msg = autoware_vehicle_msgs::msg::ControlModeReport();
    control_mode_report_msg.stamp = this->now();
    control_mode_report_msg.mode = msg->control_mode;
    control_mode_pub_->publish(control_mode_report_msg);
    // Actuation Status Publisher
    auto actuation_status_msg = tier4_vehicle_msgs::msg::ActuationStatusStamped();
    actuation_status_msg.header.stamp = this->now();
    actuation_status_msg.status.accel_status = msg->pedal_pos_engine;
    actuation_status_msg.status.brake_status = msg->pedal_pos_break;
    actuation_status_msg.status.steer_status = msg->steering_wheel_angle;
    actuation_status_pub_->publish(actuation_status_msg);
    // ...

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<M8_Vehicle_Interface>());
    rclcpp::shutdown();
    return 0;
}