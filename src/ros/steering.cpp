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

#include "bridge.h"
#include "logger.h"
#include "steering_angle_publisher.h"
#include "steering_angle_subscriber.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"
#include "steering_config.h"
 
#include <thread>
#include <chrono>
#include <memory>

// #define BUSNAME ... // set by CMake
// #define BAUDRATE ... // set by CMake

int main(int argc, char* argv[]) {

	// Set the name of your CAN bus. "slcan0" is a common bus name
	// for the first SocketCAN device on a Linux system.
	const std::string busname = "can0";

	// Set the baudrate of your CAN bus. Most drivers support the values
	// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
	const std::string baudrate = "1M";

	const double loop_rate = 10; // [Hz]

	kaco::Master master;
	if (!master.start(busname, baudrate)) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	//master.core.nmt.reset_all_nodes();

	std::this_thread::sleep_for(std::chrono::seconds(1));
	size_t num_devices_required = 1;
	while (master.num_devices()<num_devices_required) {
		ERROR("Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
		PRINT("Trying to discover more nodes via NMT Node Guarding...");
		master.core.nmt.discover_nodes();
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Create bridge
	ros::init(argc, argv, "canopen_bridge");
	kaco::Bridge bridge;

	bool found = false;

	if (master.num_devices() > 1) {
		ERROR("Found more than 1 device, this shouldn't happen");
		return EXIT_FAILURE;
	}

	kaco::Device& device = master.get_device(0);
	device.start();

	device.load_dictionary_from_library();

	const auto profile = device.get_device_profile_number();
	if (device.get_node_id() != 65) {
		PRINT("Wrong node ID for the motor!");
		return EXIT_FAILURE;
	}
	PRINT("Found CiA "<<std::dec<<(unsigned)profile<<" device with node ID 65: "<<device.get_entry("manufacturer_device_name"));

	if (profile==402) {

		found = true;
			
		device.execute("initialise_motor");
		// TODO: target_position should be mapped to a PDO

		auto jspub = std::make_shared<kaco::SteeringAnglePublisher>(device, M_0_DEG_STEP, M_360_DEG_STEP);
		bridge.add_publisher(jspub, loop_rate);
			
		auto jssub = std::make_shared<kaco::SteeringAngleSubscriber>(device, M_0_DEG_STEP, M_360_DEG_STEP);
		bridge.add_subscriber(jssub);

	}

	if (!found) {
		ERROR("This example is intended for use with a CiA 402 device but I can't find one.");
		return EXIT_FAILURE;
	}
	PRINT("Bridge running");
	bridge.run();

}
