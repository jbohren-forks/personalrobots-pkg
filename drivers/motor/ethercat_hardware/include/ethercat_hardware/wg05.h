/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef WG05_H
#define WG05_H

#include <ethercat_hardware/motor_control_board.h>

struct WG05Status {
	uint16_t device_type_id;
	uint16_t device_rev;
	uint8_t  mode;
	uint8_t  digital_out;
	uint16_t pwm_duty;
	uint16_t programmed_current;
	uint8_t  current_loop_kp;
	uint8_t  current_loop_ki;
	uint16_t measured_current;
	uint16_t empty1;
	uint32_t timestamp;
	uint32_t encoder_pos;
	uint32_t encoder_index_pos;
	uint16_t encoder_invalid_count;
	uint8_t  encoder_status;
	uint8_t  limit_status;
	uint32_t limit_on_off_pos;
	uint32_t limit_off_on_pos;
	uint16_t supply_voltage;
	uint16_t motor_voltage;
	uint16_t board_temperature;
	uint16_t bridge_temperature;
	uint8_t  pdo_command_irq_count;
	uint8_t  mbx_command_irq_count;
	uint16_t ecat_packet_count;
	uint16_t pdi_timeout_error_count;
	uint16_t pdi_checksum_error_count;;
	uint8_t  empty2;
	uint8_t  checksum;
} __attribute__ ((__packed__));

typedef WG05Status WG05Command;

class WG05: public MotorControlBoard {
	static const int STATUS_PHY_ADDR = 0x2000;
	static const int COMMAND_PHY_ADDR = 0x1000;

	enum {
		MODE_OFF, MODE_PWM, MODE_PID
	};

public:
	WG05() : MotorControlBoard(WG05_PRODUCT_CODE, sizeof(WG05Command), sizeof(WG05Status)) {}
	void configure(int &startAddress, EtherCAT_SlaveHandler *sh);
    void convertCommand(ActuatorCommand &command, unsigned char *buffer);
    void convertState(ActuatorState &state, unsigned char *buffer);

private:
	static const EC_UDINT WG05_PRODUCT_CODE = 0x57473035;
};

#endif /* WG05_H */
