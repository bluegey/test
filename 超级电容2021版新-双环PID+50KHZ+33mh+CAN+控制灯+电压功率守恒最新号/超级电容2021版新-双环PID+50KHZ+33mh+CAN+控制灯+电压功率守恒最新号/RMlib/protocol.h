/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
/** @file protocol.h
 *  @version 1.0
 *  @date Oct 2017
 *
 *  @brief the communication protocol of main control with 
 *         judge system and computer
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f10x.h"                  // Device header
#include "judgement_info.h"

#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#define PROTOCAL_FRAME_MAX_SIZE  200

/** 
  * @brief  frame header structure definition
  */
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef __packed struct
{
	frame_header_t		FrameHeader;
	judge_data_id_e		CmdID;
//	ext_student_interactive_header_data_t interactive_data;
  __packed union
	{ 
		ext_game_robot_state_t game_information;
  ext_robot_hurt_t  blood_changed_data;
  ext_shoot_data_t       real_shoot_data;
	ext_power_heat_data_t   real_powerheat_data;
  ext_event_data_t      rfid_data;
  ext_game_result_t      game_result_data;
  ext_buff_musk_t         get_buff_data;
	ext_game_robot_pos_t		 gameRobotPos;
		client_show_data_t  client_show_data;
	}Data;
	uint16_t		CRC16;	//所有数据CRC校验
}Dateframe_t;//数据帧

uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

void append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
void append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);


#endif
