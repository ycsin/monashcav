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
#include "steering_torque_subscriber.h"
#include "utils.h"
#include "logger.h"
#include "bridge.h"
#include "profiles.h"
#include "sdo_error.h"
#include "steering_config.h"

#include "ros/ros.h"

#include <string>

namespace kaco {

SteeringTorqueSubscriber::SteeringTorqueSubscriber(Device& device, int32_t position_0_degree,
	int32_t position_360_degree)
	: m_device(device), m_position_0_degree(position_0_degree),
		m_position_360_degree(position_360_degree), m_initialized(false)
{

	const uint16_t profile = device.get_device_profile_number();

	if (profile != 402) {
		throw std::runtime_error("SteeringTorqueSubscriber can only be used with a CiA 402 device."
			" You passed a device with profile number "+std::to_string(profile));
	}

}

void SteeringTorqueSubscriber::advertise() {

	assert(!m_topic_name.empty());
	DEBUG_LOG("Advertising "<<m_topic_name);
	ros::NodeHandle nh;
	m_subscriber = nh.subscribe(m_topic_name, queue_size, &SteeringTorqueSubscriber::receive, this);
	m_initialized = true;

}

void SteeringTorqueSubscriber::receive(const monashcav::Steer& msg) {
	const int32_t pos = msg.steering_angle;
	DEBUG_LOG("Received SteeringAngle message");
	DEBUG_DUMP(pos);
	DEBUG_DUMP(msg.steering_angle);
	DEBUG_DUMP(msg.steering_enable);
	DEBUG_DUMP(msg.steering_error);

	if (msg.steering_angle <= 1000 && msg.steering_angle >= 0) {
		if (msg.steering_enable) {
			if (m_device.getState() == MOTOR_READY) {
				m_device.execute("enable_motor");
				m_device.setRunning();
				std::this_thread::sleep_for(std::chrono::milliseconds(300));
				PRINT("Enabled operation, motor running");
			}
			try {
				if(m_device.getErrorCode() == M_ERR_CONNECTION_LOST) {
					Bridge bridge;
					auto jspub = std::make_shared<SteeringTorquePublisher>(m_device, M_0_DEG_STEP, M_360_DEG_STEP);
					bridge.add_publisher(jspub, loop_rate);

					auto jssub = std::make_shared<kaco::SteeringTorqueSubscriber>(m_device, M_0_DEG_STEP, M_360_DEG_STEP);
					bridge.add_subscriber(jssub);
				}
				m_device.set_entry("Target_torque_x1000",(int16_t)pos);
				
			} catch (const sdo_error& error) {
				// TODO: only catch timeouts?
				ERROR("Exception in SteeringTorqueSubscriber::receive(): "<<error.what());
			}
		} else {
			m_device.execute("disable_operation");
			m_device.setReady();
			PRINT("Disabled operation, motor ready");
		}
	} else {
		ERROR("Exception in SteeringTorqueSubscriber::receive(): Input angle violates limits");
		m_device.setErrorCode(M_ERR_EXCEEDS_LIMIT);
	}
}

} // end namespace kaco


