/**
 * Manages the communication between the Jetson and the Teensie through JSON communication
*/
#include "crobot_hardware/crobot_hardware_system.hpp"
#include <string>
#include <algorithm>
#include <nlohmann/json.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using json = nlohmann::json;

using namespace std;

namespace crobot_hardware
{
    /**
     * Initalizes the hardware interface
     * 
     * @param info the robot's information, passed in from TODO: INCLUDE LOCATION HERE
    */
    hardware_interface::CallbackReturn CrobotHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.wheel_fl_name = info_.hardware_parameters["front_left_wheel_name"];
        cfg_.wheel_fr_name = info_.hardware_parameters["front_right_wheel_name"];
        cfg_.wheel_bl_name = info_.hardware_parameters["back_left_wheel_name"];
        cfg_.wheel_br_name = info_.hardware_parameters["back_right_wheel_name"];

        cfg_.ankle_fl_name = info_.hardware_parameters["front_left_ankle_name"];
        cfg_.ankle_fr_name = info_.hardware_parameters["front_right_ankle_name"];
        cfg_.ankle_bl_name = info_.hardware_parameters["back_left_ankle_name"];
        cfg_.ankle_br_name = info_.hardware_parameters["back_right_ankle_name"];

        cfg_.sweeper_name = info_.hardware_parameters["sweeper_name"];
        cfg_.winch_name = info_.hardware_parameters["winch_name"];

        // Declared what I think I need to start defining the things
        cfg_.shoulder_name = info_.hardware_parameters["shoulder_name"];
        cfg_.elbow_name = info_.hardware_parameters["elbow_name"];
        cfg_.gripper_name = info_.hardware_parameters["gripper_name"];
        cfg_.flagdropper_name = info_.hardware_parameters["flagdropper_name"];

        cfg_.imu_name = info_.hardware_parameters["imu_name"];

        cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
        cfg_.device = info_.hardware_parameters["dev"];
        cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
        cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

        wheels_.resize(4);
        wheels_[0].name = cfg_.wheel_fl_name;
        wheels_[1].name = cfg_.wheel_fr_name;
        wheels_[2].name = cfg_.wheel_bl_name;
        wheels_[3].name = cfg_.wheel_br_name;

        ankles_.resize(4);
        ankles_[0].name = cfg_.ankle_fl_name;
        ankles_[1].name = cfg_.ankle_fr_name;
        ankles_[2].name = cfg_.ankle_bl_name;
        ankles_[3].name = cfg_.ankle_br_name;

        sweeper_.name = cfg_.sweeper_name;
        winch_.name = cfg_.winch_name;

        // More definitions (no clue what these do, it's 4am lol)
        shoulder_.name = cfg_.shoulder_name;
        elbow_.name = cfg_.elbow_name;
        gripper_.name = cfg_.gripper_name;
        flagdropper_.name = cfg_.flagdropper_name;

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * Export the current state of the wheels
     * 
     * @return the current state of the wheel, as a array vector of hardware_interface::StateInterface objects
    */
    std::vector<hardware_interface::StateInterface> CrobotHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto &wheel : wheels_)
        {
            if (wheel.name.find("front") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.vel));

                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    wheel.name, hardware_interface::HW_IF_POSITION, &wheel.pos));
            }
        }

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            cfg_.imu_name, hardware_interface::HW_IF_VELOCITY, &imu_vel));


        return state_interfaces;
    }
    /**
     * Exports the command interface for the robot to run commands
     * 
     * @return the current command interfaces, as a array vector of hardware_interface::CommandInterface objects
    */
    std::vector<hardware_interface::CommandInterface> CrobotHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto &wheel : wheels_)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                wheel.name, hardware_interface::HW_IF_VELOCITY, &wheel.cmd));
        }

        for (auto &ankle : ankles_)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                ankle.name, hardware_interface::HW_IF_POSITION, &ankle.cmd));
        }

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            sweeper_.name, hardware_interface::HW_IF_POSITION, &sweeper_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            winch_.name, hardware_interface::HW_IF_VELOCITY, &winch_.cmd));

        // Really out of my depth now, but I don't think I'm doing things super wrong
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            shoulder_.name, hardware_interface::HW_IF_VELOCITY, &shoulder_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            elbow_.name, hardware_interface::HW_IF_VELOCITY, &elbow_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            gripper_.name, hardware_interface::HW_IF_VELOCITY, &gripper_.cmd));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            flagdropper_.name, hardware_interface::HW_IF_VELOCITY, &flagdropper_.cmd));

        return command_interfaces;
    }

    /**
     * Configures the robot. Runs after on_init()
     * 
     * @param previous_state the previous state of the robot. Unused since we do not track the current
     *                       state of the robot between shutdowns.
     * @return SUCCESS if successful, ERROR otherwise
    */
    hardware_interface::CallbackReturn CrobotHardware::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Configuring...");

        

        if (!serial_comm_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms))
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("CrobotHardware"),
                "Failed to connect to device '%s' at %d baud",
                cfg_.device.c_str(),
                cfg_.baud_rate);

            return hardware_interface::CallbackReturn::ERROR;
        }

        // Create a node for publishing IMU data
        imu_node_ = rclcpp::Node::make_shared("crobot_imu_publisher");
        imu_pub_ = imu_node_->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/raw", rclcpp::SensorDataQoS());

        mode_node_ = rclcpp::Node::make_shared("crobot_embedded_mode_node");
        mode_sub_ = mode_node_->create_subscription<std_msgs::msg::String>("/crobot_embedded_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "craterRun") {
                    embedded_mode_ = EmbeddedMode::CRATER_RUN;
                }
                else if (msg->data == "launchDrone") {
                    embedded_mode_ = EmbeddedMode::LAUNCH_DRONE;
                }
                else if (msg->data == "openArm") {
                    embedded_mode_ = EmbeddedMode::OPEN_ARM;
                }
                else if (msg->data == "closeArm") {
                    embedded_mode_ = EmbeddedMode::CLOSE_ARM;
                }
                else if (msg->data == "write") {
                    embedded_mode_ = EmbeddedMode::NORMAL;
                }
            });
        photoresistor_node_ = rclcpp::Node::make_shared("crobot_photoresistor_publisher");
        photoresistor_pub_  = photoresistor_node_->create_publisher<std_msgs::msg::Int32>(
            "/photoresistor", rclcpp::SensorDataQoS());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * Cleanups the hardware interface on shutdown
     * 
     * @param previous_state the previous state of the robot. Unused
     * @return SUCCESS if successful, ERROR otherwise
    */
    hardware_interface::CallbackReturn CrobotHardware::on_cleanup(
        const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Cleaning up...");

        serial_comm_.disconnect();

        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"), "Cleaned up.");

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    /**
     * Runs once when the robot is activated. Sets the starting values of the robot. Runs after on_configure()
     * 
     * @param previous_state the previous state of the robot. Unused
     * @return SUCCESS if successful, ERROR otherwise
    */
    hardware_interface::CallbackReturn CrobotHardware::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        serial_comm_.clearBuffers();
        first_read_ = true; // Reset first read flag on activation
        last_ticks_fl_ = 0;
        last_ticks_fr_ = 0;

        for (int i = 0; i < 4; ++i)
        {
            wheels_[i].pos = 0.0;
            wheels_[i].vel = 0.0;
            wheels_[i].cmd = 0.0;

            ankles_[i].pos = 0.0;
            ankles_[i].cmd = 0.0;
        }

        sweeper_.pos = 0.0;
        sweeper_.cmd = 0.0;

        winch_.vel = 0.0;
        winch_.cmd = 0.0;

        shoulder_.pos = 0.0;
        shoulder_.cmd = 0.0;

        elbow_.pos = 0.0;
        elbow_.cmd = 0.0;

        gripper_.pos = 0.0;
        gripper_.cmd = 0.0;

        flagdropper_.pos = 0.0;
        flagdropper_.cmd = 0.0;

        return hardware_interface::CallbackReturn::SUCCESS;
    }
    /**
     * Runs once when the robot is shutdown.
     * 
     * @param previous_state the previous state of the robot. Unused
     * @return SUCCESS if successful, ERROR otherwise
    */
    hardware_interface::CallbackReturn CrobotHardware::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        // TODO: Implement deactivation
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    /**
     * Reads the data from the Teensie in order to update the state interfaces of the robot.
     * Data is transferred as a JSON file from the Teensie to the Jetson
     * 
     * @param time the current elapsed time of the robot since startup
     * @param period the current elapsed time of the hardawre interface since communication with the Teensie
     * 
     * @return SUCCESS if successful, ERROR otherwise
    */
    hardware_interface::return_type CrobotHardware::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (!serial_comm_.isConnected())
        {
            return hardware_interface::return_type::ERROR;
        }

        //reads the JSON file from the teensie as a string.
        std::string line = serial_comm_.readLine();

        if (!line.empty())
        {
            try
            {
                json response = json::parse(line);

                if (response.contains("encoders"))
                {
                    //the gear ratio. 4096 internal rotation for one wheel rotation
                    const double COUNTS_PER_REV = 4096.0;
                    const double TWO_PI = 2.0 * M_PI;
                    const double dt = period.seconds();

                    int32_t ticks_fl = response["encoders"]["front_left"];
                    int32_t ticks_fr = response["encoders"]["front_right"];
                    // int32_t ticks_br = response["encoders"]["back_right"];

                    
                    wheels_[0].pos = (ticks_fl / COUNTS_PER_REV) * TWO_PI;
                    wheels_[1].pos = (ticks_fr / COUNTS_PER_REV) * TWO_PI;
                    // wheels_[2].pos = wheels_[0].pos;
                    // wheels_[3].pos = wheels_[1].pos;

                    if (!first_read_ && dt > 0.0)
                    {
                        wheels_[0].vel = ((ticks_fl - last_ticks_fl_) / COUNTS_PER_REV) * TWO_PI / dt;
                        wheels_[1].vel = ((ticks_fr - last_ticks_fr_) / COUNTS_PER_REV) * TWO_PI / dt;
                        // wheels_[3].vel = ((ticks_br - last_ticks_br_) / COUNTS_PER_REV) * TWO_PI / dt;
                        wheels_[2].vel = wheels_[0].vel;
                    }

                    //ensures that the wheel positions aren't accidently updated twice in one tick
                    last_ticks_fl_ = ticks_fl;
                    last_ticks_fr_ = ticks_fr;
                    first_read_ = false;
                }

                double raw_yaw_rate_rad = response["yaw"];

                // Collect stationary bias samples at startup
                if (!bias_calibrated_)
                {
                    imu_yaw_bias_ += raw_yaw_rate_rad;
                    bias_sample_count_++;
                    if (bias_sample_count_ >= BIAS_SAMPLES)
                    {
                        imu_yaw_bias_ /= BIAS_SAMPLES;
                        bias_calibrated_ = true;
                        RCLCPP_INFO(rclcpp::get_logger("CrobotHardware"),
                                    "IMU yaw bias calibrated: %.6f rad/s", imu_yaw_bias_);
                    }
                }

                double corrected_yaw_rate = raw_yaw_rate_rad - (bias_calibrated_ ? imu_yaw_bias_ : 0.0);
                imu_vel = corrected_yaw_rate; // keep state interface working too

                // Publish sensor_msgs/Imu
                auto imu_msg = sensor_msgs::msg::Imu();
                imu_msg.header.stamp = time;
                imu_msg.header.frame_id = "base_link"; // must match your URDF

                imu_msg.angular_velocity.x = 0.0;
                imu_msg.angular_velocity.y = 0.0;
                imu_msg.angular_velocity.z = corrected_yaw_rate;

                // Tell EKF the variance on omega_z (~0.01 rad²/s² is reasonable for a decent IMU)
                imu_msg.angular_velocity_covariance[8] = 0.0000001;

                // Mark orientation and linear accel as unknown (diagonal = -1 means "don't use")
                imu_msg.orientation_covariance[0] = -1.0;
                imu_msg.linear_acceleration_covariance[0] = -1.0;

                imu_pub_->publish(imu_msg);

                // Spin the imu_node_ so it actually sends
                rclcpp::spin_some(imu_node_);

                int photoresistor_val = response["photoresistor"];

                auto photoresistor_msg = std_msgs::msg::Int32();

                photoresistor_msg.data = photoresistor_val;

                photoresistor_pub_->publish(photoresistor_msg);

                rclcpp::spin_some(photoresistor_node_);
            }
            catch (json::parse_error &e)
            {
                RCLCPP_WARN(rclcpp::get_logger("CrobotHardware"), "Bad serial packet: %s, raw string: %s", e.what(), line.c_str());
                return hardware_interface::return_type::OK;
            }
        }
        return hardware_interface::return_type::OK;
    }

    /**
     * Writes to the teensie to execute commands
     * 
     * @param time the current elapsed time of the robot since startup
     * @param period the current elapsed time of the hardawre interface since communication with the Teensie
     * 
     * @return SUCCESS if successful, ERROR otherwise
    */
    hardware_interface::return_type CrobotHardware::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (mode_node_) {
            rclcpp::spin_some(mode_node_);
        }
        if (!serial_comm_.isConnected())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"),
                         "Cannot write to hardware: not connected");
            return hardware_interface::return_type::ERROR;
        }

        json j;

        /* Writes to the "cmd" key for the teensie to execute*/
        if (embedded_mode_ == EmbeddedMode::CRATER_RUN) {
            j["cmd"] = "craterRun";
        } else if (embedded_mode_ == EmbeddedMode::LAUNCH_DRONE) {
            j["cmd"] = "launchDrone";
        } else if (embedded_mode_ == EmbeddedMode::OPEN_ARM) {
            j["cmd"] = "openArm";
        } else if (embedded_mode_ == EmbeddedMode::CLOSE_ARM) {
            j["cmd"] = "closeArm";
        } else {
            j["cmd"] = "write";

            const double RAD_TO_DEG = 180.0 / M_PI;
            const float hardware_conversion_factor = 0.9;
            /* Writes to the wheel states */
            j["ankles"]["front_left"] = (int)(130.0 + ankles_[0].cmd * RAD_TO_DEG / hardware_conversion_factor);
            j["ankles"]["front_right"] = (int)(53.0 + ankles_[1].cmd * RAD_TO_DEG / hardware_conversion_factor);
            j["ankles"]["back_left"] = (int)(57.0 + ankles_[2].cmd * RAD_TO_DEG / hardware_conversion_factor);
            j["ankles"]["back_right"] = (int)(110.0 + ankles_[3].cmd * RAD_TO_DEG / hardware_conversion_factor);

            const double MAX_WHEEL_SPEED = cfg_.max_wheel_speed_meters / cfg_.wheel_radius; // r/s corresponding to full command (255)

            j["wheels"]["front_left"] = std::clamp((int)(wheels_[0].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);
            j["wheels"]["front_right"] = std::clamp((int)(wheels_[1].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);
            j["wheels"]["back_left"] = std::clamp((int)(wheels_[2].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);
            j["wheels"]["back_right"] = std::clamp((int)(wheels_[3].cmd / MAX_WHEEL_SPEED * 255.0), -255, 255);

            j["sweeper"] = (int)(40.0 + sweeper_.cmd * RAD_TO_DEG);
            j["winch"] = std::clamp((int)(winch_.cmd * 255.0), -255, 255);

            j["flag"] = (int)(80.0 + flagdropper_.cmd * RAD_TO_DEG); // initial value + angle change(?)
            j["shoulder"] = (int)(shoulder_.cmd * RAD_TO_DEG);       // Just our desired angle?
            j["elbow"] = (int)(180.0 - shoulder_.cmd * RAD_TO_DEG);   // initial value - angle change(?)
            j["gripper"] = (int)(shoulder_.cmd * RAD_TO_DEG);     // open to angle set
            j["flag"] = (int)(80.0 + flagdropper_.cmd * RAD_TO_DEG);
        }

        std::string j_str = j.dump() + "\n";

        /* Sends the JSON file over to the hardware through UART (?)*/

        int bytesSent = serial_comm_.writeBytes(j_str.c_str(), j_str.size());

        if (bytesSent != (int)j_str.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("CrobotHardware"),
                         "Sent %d bytes, expected to send %zu bytes",
                         bytesSent, j_str.size());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    crobot_hardware::CrobotHardware, hardware_interface::SystemInterface)