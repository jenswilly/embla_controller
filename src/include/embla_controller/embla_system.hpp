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

// Hardware System interface for the JWR-02 Embla robot
// This code is based on the ros2_control_demo_example_2 package

#ifndef EMBLA_SYSTEM_HPP_
#define EMBLA_SYSTEM_HPP_

// If next line is commented out, no hardware communication is attempted
// #define USE_HARDWARE 1
#ifndef USE_HARDWARE
#pragma message "Compiling WITHOUT hardware support. Define USE_HARDWARE in embla_system.hpp to enable"
#endif

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#ifdef USE_HARDWARE
#include "roboclaw/roboclaw_driver.h"
#endif

// NB: This package does _not_ support building on Windows
// #include "embla_controller/visibility_control.h"

namespace embla_controller
{
    class EmblaSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(EmblaSystemHardware)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // Store the command for the simulated robot
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<int> left_side_indices_, right_side_indices_; // Used to keep track of which joints are on the left and right side of the robot
#ifdef USE_HARDWARE
        roboclaw::driver *roboclaw_driver_;
#endif
        uint8_t roboclaw_address_;
        int pulses_per_rev_;

        double encoderPulsesToAngular(const int &encoder) const;
        int angularToEncoderPulses(const double &angle) const;
    };

} // namespace embla_controller

#endif // EMBLA_SYSTEM_HPP_