// The MIT License (MIT)

// Copyright (c) 2024 Jens Willy Johannsen <jens@jwrobotics.com>

//  Permission is hereby granted, free of charge, to any person obtaining a
//  copy of this software and associated documentation files (the "Software"),
//  to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense,
//  and/or sell copies of the Software, and to permit persons to whom the
//  Software is furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
//  OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
//  DEALINGS IN THE SOFTWARE.

// Hardware controller for the JWR-02 Embla robot
// This code is based on the ros2_control_demo_example_2 package

// File is in hardware/include/embla_controller/
// Include path is defined with target_include_directories() in CMakeLists.txt
#include "embla_controller/embla_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace embla_controller
{
    hardware_interface::CallbackReturn EmblaSystemHardware::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_positions_.resize(this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_velocities_.resize(this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_.resize(this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        // We expect exactly two joints
        if (info_.joints.size() != 2)
        {
            RCLCPP_FATAL(rclcpp::get_logger("EmblaSystemHardware"), "Expected 2 joints, got %zu", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate the joint information
        // The system has exactly two states (POSITION and VELOCITY) and one command interface (VELOCITY) on each joint
        for (const hardware_interface::ComponentInfo &joint : this->info_.joints)
        {
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EmblaSystemHardware"),
                    "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                    joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EmblaSystemHardware"),
                    "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
                    joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EmblaSystemHardware"),
                    "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EmblaSystemHardware"),
                    "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("EmblaSystemHardware"),
                    "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
                    joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> EmblaSystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (auto i = 0u; i < this->info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                this->info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                this->info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> EmblaSystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (auto i = 0u; i < this->info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                this->info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn EmblaSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Initialize RoboClaw
        roboclaw_driver_ = new roboclaw::driver("/dev/roboclaw", 460800);
        if (!roboclaw_driver_->serial->isOpen())
        {
            RCLCPP_WARN(rclcpp::get_logger("EmblaSystemHardware"), "Roboclaw port not open - waiting");
            while (!roboclaw_driver_->serial->isOpen())
                ;
        }

        // Read current encoder values
        std::pair<int, int> encoders = roboclaw_driver_->get_encoders(0x80);
        hw_positions_[0] = encoders.first;
        hw_positions_[1] = encoders.second;

        // set some default values
        for (auto i = 0u; i < hw_positions_.size(); i++)
        {
            if (std::isnan(hw_positions_[i]))
            {
                hw_velocities_[i] = 0;
                hw_commands_[i] = 0;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("EmblaSystemHardware"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn EmblaSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Deactivate RoboClaw driver
        delete roboclaw_driver_;

        RCLCPP_INFO(rclcpp::get_logger("EmblaSystemHardware"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type EmblaSystemHardware::read(
        const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
    {
        std::ignore = period;

        auto encoders = roboclaw_driver_->get_encoders(0x80);
        auto velocities = roboclaw_driver_->get_velocity(0x80);

        hw_positions_[0] = encoders.first;
        hw_positions_[1] = encoders.second;
        hw_velocities_[0] = velocities.first;
        hw_velocities_[1] = velocities.second;

        for (std::size_t i = 0; i < hw_velocities_.size(); i++)
        {
            RCLCPP_INFO(
                rclcpp::get_logger("EmblaSystemHardware"),
                "Got position state %.5f and velocity state %.5f for '%s'!", hw_positions_[i],
                hw_velocities_[i], this->info_.joints[i].name.c_str());
        }

        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        /*
        for (std::size_t i = 0; i < hw_velocities_.size(); i++)
        {
            // Simulate wheels' movement as a first-order system
            // Update the joint status: this is a revolute joint without any limit.
            // Simply integrates
            hw_positions_[i] = hw_positions_[i] + period.seconds() * hw_velocities_[i];
        }
        */
        // END: This part here is for exemplary purposes - Please do not copy to your production code

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type embla_controller::EmblaSystemHardware::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
        RCLCPP_INFO(rclcpp::get_logger("EmblaSystemHardware"), "Writing...");

        // Send velocity commands to the hardware
        roboclaw_driver_->set_velocity(0x80, std::make_pair(hw_commands_[0], hw_commands_[1]));

        // Optimistically set the velocities to the commands
        hw_velocities_[0] = hw_commands_[0];
        hw_velocities_[1] = hw_commands_[1];

        /*
        for (auto i = 0u; i < hw_commands_.size(); i++)
        {
            // Simulate sending commands to the hardware
            RCLCPP_INFO(
                rclcpp::get_logger("EmblaSystemHardware"), "Got command %.5f for '%s'!", hw_commands_[i],
                this->info_.joints[i].name.c_str());

            // hw_velocities_[i] = hw_commands_[i];
        }
        RCLCPP_INFO(rclcpp::get_logger("EmblaSystemHardware"), "Joints successfully written!");
        // END: This part here is for exemplary purposes - Please do not copy to your production code
        */

        return hardware_interface::return_type::OK;
    }

} // namespace embla_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    embla_controller::EmblaSystemHardware, hardware_interface::SystemInterface)