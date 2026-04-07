#include <m8_vehicle_interface.hpp>

void M8_Vehicle_Interface::vehicle_set_turn_lights(uint8_t command) {
    if (command != prev_turnlights_command) {
        auto turn_lights_request = std::make_shared<zgw_interfaces::srv::VehicleSetIndicator::Request>();
        turn_lights_request->signal = command > 2 /*3 = Hazard*/ ? 2 : command < 2 /* 0 or 1*/? command : /*2 = Right*/ 3;
        turn_lights_request->header.stamp = this->now();
        auto result = turn_lights_command_client->async_send_request(turn_lights_request);
    }
}

std::pair<double,double> M8_Vehicle_Interface::calculate_torque_values(float velocity_desired) {
    std::pair<double, double> carla_return_values(0.0, 0.0);
    if(autoware_emergency) { // emergency brake
        brake_torque = -2200.0;
        engine_torque = 0.0;
        e_v = inte_v = deri_v = e_v_prev = acc = 0.0;
        // brake also in Carla?
    }
    else if(velocity_desired <= 0.0001) { // nearly stand still
        brake_torque = -1400.0;
        engine_torque = 0.0;
        e_v = inte_v = deri_v = e_v_prev = acc = 0.0;
        // brake also in Carla?
    } else { // normal driving
        e_v = velocity_desired - v; // velocity error term
        inte_v += e_v * st; // integral term
        deri_v = (e_v + e_v_prev) / st; // derivate term

        acc = Kp * e_v + Ki * inte_v + Kd * deri_v; // + a_ref; // acceleration
 
        double a_max_acc = 8.68; // m/s² (BMW M8 max acceleration)
        double a_max_dec = -11.71; // m/s² (BMW M8 max deceleration)

        double acc_cmd = std::clamp(acc, a_max_dec, a_max_acc); // limit acceleration to [a_max_dec, a_max_acc] m/s² (BMW M8 max deceleration and acceleration)
 
        if(acc_cmd >= 0) { // acceleration
            carla_return_values.first = (acc_cmd / a_max_acc);
            // calculate engine torque
            engine_torque = (acc_cmd / a_max_acc) * max_engine_torque; // scale to max engine torque
            brake_torque = 0.0; // no brake torque when accelerating
        } else { // deceleration
            carla_return_values.second = (acc_cmd / a_max_dec);
            engine_torque = 0.0; // no engine torque
            brake_torque = (acc_cmd / a_max_dec) * -max_engine_torque; // scale to max engine torque
        }
    }
    return carla_return_values;
}