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
 
#include "steering_torque_publisher.h"
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

SteeringTorquePublisher::SteeringTorquePublisher(Device& device, int32_t position_0_degree,
	int32_t position_360_degree, const std::string& position_actual_field)
	: m_device(device), m_position_0_degree(position_0_degree),
		m_position_360_degree(position_360_degree), m_position_actual_field(position_actual_field), m_initialized(false)
{

	const uint16_t profile = device.get_device_profile_number();

	if (profile != 402) {
		throw std::runtime_error("SteeringTorquePublisher can only be used with a CiA 402 device."
			" You passed a device with profile number "+std::to_string(profile));
	}

}

void SteeringTorquePublisher::advertise() {

	assert(!m_topic_name.empty());
	DEBUG_LOG("Advertising "<<m_topic_name);
	ros::NodeHandle nh;
	m_publisher = nh.advertise<monashcav::Steer>(m_topic_name, queue_size);
	m_initialized = true;

}

void SteeringTorquePublisher::publish() {

	try {

		if (!m_initialized) {
			throw std::runtime_error("[SteeringTorquePublisher] publish() called before advertise().");
		}

		monashcav::Steer js;

		const int32_t pos = m_device.get_entry("Torque_actual_x1000");
		js.steering_angle = pos;
		js.steering_enable = m_device.getState();
		js.steering_error = m_device.getErrorCode();

		DEBUG_LOG("Sending Steering Torque message");
		DEBUG_DUMP(pos);
		DEBUG_DUMP(js.steering_angle);

		m_publisher.publish(js);
		m_device.setErrorCode(M_ERR_NONE);
	} catch (const sdo_error& error) {
		// TODO: only catch timeouts?
		ERROR("Exception in SteeringTorquePublisher::publish(): "<<error.what());
		ERROR("Connetion to stepper motor lost");
		m_device.setErrorCode(M_ERR_CONNECTION_LOST);
		monashcav::Steer js;
		js.steering_angle = 0;
		js.steering_enable = 0;
		js.steering_error = M_ERR_CONNECTION_LOST;
		m_publisher.publish(js);
		m_device.setReady();
		m_device.execute("initialise_motor");
	}

}

} // end namespace kaco


