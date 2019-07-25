/*
 * Copyright (c) 2015-2016, Thomas Keh
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
#include "steering_angle_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "profiles.h"
#include "sdo_error.h"
#include "steering_config.h"

#include "ros/ros.h"

#include <string>

namespace kaco {

SteeringAngleSubscriber::SteeringAngleSubscriber(Device& device, int32_t position_0_degree,
	int32_t position_360_degree)
	: m_device(device), m_position_0_degree(position_0_degree),
		m_position_360_degree(position_360_degree), m_initialized(false)
{

	const uint16_t profile = device.get_device_profile_number();

	if (profile != 402) {
		throw std::runtime_error("SteeringAnglePublisher can only be used with a CiA 402 device."
			" You passed a device with profile number "+std::to_string(profile));
	}

	const Value operation_mode = device.get_entry("Modes of operation display");

	// TODO: look into INTERPOLATED_POSITION_MODE
	if (operation_mode != Profiles::constants.at(402).at("profile_position_mode")
		&& operation_mode != Profiles::constants.at(402).at("interpolated_position_mode")) {
		throw std::runtime_error("[JointStatePublisher] Only position mode supported yet."
			" Try device.set_entry(\"modes_of_operation\", device.get_constant(\"profile_position_mode\"));");
	}

}

void SteeringAngleSubscriber::advertise() {

	assert(!m_topic_name.empty());
	DEBUG_LOG("Advertising "<<m_topic_name);
	ros::NodeHandle nh;
	m_subscriber = nh.subscribe(m_topic_name, queue_size, &SteeringAngleSubscriber::receive, this);
	m_initialized = true;

}

void SteeringAngleSubscriber::receive(const kacanopen::Steer& msg) {
	const int32_t pos = steering_to_motor_pos(msg.steering_angle);
	static bool motor_enabled = 0;

	DEBUG_LOG("Received SteeringAngle message");
	DEBUG_DUMP(pos);
	DEBUG_DUMP(msg.steering_angle);
	
	if (m_device.getState() == MOTOR_READY && msg.steering_angle <= 1080 && msg.steering_angle >= -1080) {
		m_device.execute("enable_operation");
		m_device.setRunning();
		PRINT("Enabled operation, motor running");
	} else if (msg.steering_angle > 1080 || msg.steering_angle < -1080) {
		m_device.execute("disable_operation");
		m_device.setReady();
		PRINT("Disabled operation, motor ready");
	}

	if (m_device.getState() == MOTOR_RUNNING) {
		try {
			m_device.execute("set_target_position_immediate",pos);
			
		} catch (const sdo_error& error) {
			// TODO: only catch timeouts?
			ERROR("Exception in SteeringAngleSubscriber::receive(): "<<error.what());
		}
	}

}

int32_t SteeringAngleSubscriber::steering_to_motor_pos(int32_t pos) const {
    const int32_t p = pos * GEAR_RATIO;
    const int32_t min = m_position_0_degree;
    const int32_t max = m_position_360_degree;
    const int32_t dist = max - min;
    const int32_t result = ((p * dist + 180) / 360) + min;
    return result;

}

} // end namespace kaco

