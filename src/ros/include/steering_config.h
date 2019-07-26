/*
 * Copyright (c) 2019-2020, Yong Cong Sin
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
 
#pragma once

#include <string>
#include <cmath>

/*
 * Stepper motor to steering wheel gear ratio
 */

#define GEAR_RATIO 4


/*
 * Publisher loop rate
 */

const double loop_rate = 10; // [Hz]

/*
 * Stepper motor config
 */

#define M_USTEP 51200
#define M_0_DEG_STEP 0
#define M_360_DEG_STEP M_USTEP
#define M_RUN_CURRENT_PERCENT 100
#define M_ACC_PROFILE 6*M_USTEP
#define M_DEC_PROFILE 6*M_USTEP
#define M_MAX_VELOCITY 3*M_USTEP
#define M_HMT_ENABLE 128
#define M_HMT_DISABLE 0
#define M_FAULT_DISABLE_DRIVE 0
#define M_FAULT_SLOW_DOWN_RAMP 1
#define M_ROTATION_LIMIT 3
#define M_ANGLE_LIMIT 360*M_ROTATION_LIMIT

/*
 * Stepper motor state
 */

#define MOTOR_READY 0
#define MOTOR_RUNNING 1

/*
 * Stepper motor error code
 */

#define M_ERR_NONE 0
#define M_ERR_REBOOTED 1
#define M_ERR_CONNECTION_LOST 2
#define M_ERR_EXCEEDS_LIMIT 3
