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
 
#include "steering_angle_publisher.h"
#include "utils.h"
#include "logger.h"
#include "profiles.h"
#include "sdo_error.h"
#include "steering_config.h"

#include "ros/ros.h"
#include "monashcav/Steer.h"

#include <string>
#include <stdexcept>

namespace kaco {

SteeringAnglePublisher::SteeringAnglePublisher(Device& device, int32_t position_0_degree,
	int32_t position_360_degree, const std::string& position_actual_field)
	: m_device(device), m_position_0_degree(position_0_degree),
		m_position_360_degree(position_360_degree), m_position_actual_field(position_actual_field), m_initialized(false)
{

	const uint16_t profile = device.get_device_profile_number();

	if (profile != 402) {
		throw std::runtime_error("JointStatePublisher can only be used with a CiA 402 device."
			" You passed a device with profile number "+std::to_string(profile));
	}

	const Value operation_mode = device.get_entry("Modes of operation display");

	// TODO: look into INTERPOLATED_POSITION_MODE
	if (operation_mode != Profiles::constants.at(402).at("profile_position_mode")
		&& operation_mode != Profiles::constants.at(402).at("interpolated_position_mode")) {
		throw std::runtime_error("[JointStatePublisher] Only position mode supported yet."
			" Try device.set_entry(\"modes_of_operation\", device.get_constant(\"profile_position_mode\"));");
		return;
	}

}

void SteeringAnglePublisher::advertise() {

	assert(!m_topic_name.empty());
	DEBUG_LOG("Advertising "<<m_topic_name);
	ros::NodeHandle nh;
	m_publisher = nh.advertise<monashcav::Steer>(m_topic_name, queue_size);
	m_initialized = true;

}

void SteeringAnglePublisher::publish() {

	try {

		if (!m_initialized) {
			throw std::runtime_error("[SteeringAnglePublisher] publish() called before advertise().");
		}

		monashcav::Steer js;

		const int32_t pos = m_device.get_entry(m_position_actual_field);
		js.steering_angle = motor_to_steering_angle(pos);
		js.steering_enable = m_device.getState();
		js.steering_error = m_device.getErrorCode();

		DEBUG_LOG("Sending Steering Angle message");
		DEBUG_DUMP(pos);
		DEBUG_DUMP(js.steering_angle);

		m_publisher.publish(js);

	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in SteeringAnglePublisher::publish(): "<<error.what());
		ERROR("Connetion to stepper motor lost");
		m_device.setErrorCode(M_ERR_CONNECTION_LOST);
		m_device.setReady();
		m_device.execute("initialise_motor");
	}

}

int32_t SteeringAnglePublisher::motor_to_steering_angle(int32_t pos) const {
    const int32_t p = pos;
    const int32_t min = m_position_0_degree;
    const int32_t max = m_position_360_degree;
    const int32_t dist = max - min;
    const int32_t result = ((p - min) * 360 + (dist * GEAR_RATIO)/2) / (dist * GEAR_RATIO);
    return result;
}

} // end namespace kaco

