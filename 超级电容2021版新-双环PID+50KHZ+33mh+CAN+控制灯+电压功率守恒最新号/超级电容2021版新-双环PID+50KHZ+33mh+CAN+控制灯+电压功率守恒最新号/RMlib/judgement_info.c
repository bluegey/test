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
/** @file judgement_info.c
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief the information from judgement system
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#include "judgement_info.h"
#include "protocol.h"
#include "CAN_Task.h"
#include "string.h"

/* data send (forward) */
/* data receive */
receive_judge_t judge_rece_mesg;
int16_t bullets_num=0;
	float shoot_speed,shoot_speed_last;
void bullet_num(void);
/**
  * @brief    get judgement system message
  */

void judgement_data_handler(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  
  switch (cmd_id)
  {
    case ROBORT_STATE_DATA_ID:
      memcpy(&judge_rece_mesg.game_information, data_addr, data_length);
    break;

    case REAL_HURT_STATE_DATA_ID:
      memcpy(&judge_rece_mesg.blood_changed_data, data_addr, data_length);
		Judgement.hurt_type=judge_rece_mesg.blood_changed_data.hurt_type;
    break;

		case REAL_HEAT_POWER_DATA_ID:
      memcpy(&judge_rece_mesg.real_powerheat_data, data_addr, data_length);
//			Judgement.Voltage =(int16_t)(judge_rece_mesg.real_powerheat_data.chassis_volt*1000);

//		Judgement.Current =(int16_t)(judge_rece_mesg.real_powerheat_data.chassis_current*1000);
//		Judgement.Power = (int16_t)(judge_rece_mesg.real_powerheat_data.chassis_power*100);
		Judgement.PowerBuffer =(int16_t)(judge_rece_mesg.real_powerheat_data.chassis_power_buffer*100);     //缓冲能量
		Judgement.ShooterHeat_17mm = judge_rece_mesg.real_powerheat_data.shooter_heat0;
		Judgement.ShooterHeat_42mm = judge_rece_mesg.real_powerheat_data.shooter_heat1;
		CAN1_Send_16bitMessage(0x301,Judgement.ShooterHeat_17mm,Judgement.level,judge_rece_mesg.real_shoot_data.bullet_speed*10,Judgement.PowerBuffer);
    break;
		
    case REAL_SHOOT_DATA_ID:
      memcpy(&judge_rece_mesg.real_shoot_data, data_addr, data_length);
    break;

    case EVENT_DATA_ID:
      memcpy(&judge_rece_mesg.rfid_data, data_addr, data_length);
    break;

//    case GAME_RESULT_ID:
//      memcpy(&judge_rece_mesg.game_result_data, data_addr, data_length);
//    break;

    case REAL_GETBUF_DATA_ID:
      memcpy(&judge_rece_mesg.get_buff_data, data_addr, data_length);
    break;
		case ROBORT_HP_DATA_ID :
			 memcpy(&judge_rece_mesg.robot_hp, data_addr, data_length);
		break;
//    case REAL_POSITION_DATA_ID:
//      memcpy(&judge_rece_mesg.gameRobotPos, data_addr, data_length);
//    break;
		
		case REAL_SUPPLY_DATA_ID:
      memcpy(&judge_rece_mesg.supply_projectile, data_addr, data_length);
				Judgement.Supply_Num =(int16_t)(judge_rece_mesg.supply_projectile.supply_projectile_num);
    break;
		default : break;
  }
  bullet_num();
}

void bullet_num()
{
	shoot_speed=judge_rece_mesg.real_shoot_data.bullet_speed;
	if(judge_rece_mesg.supply_projectile.supply_projectile_step==2)
	{
		switch (judge_rece_mesg.supply_projectile.supply_projectile_num)
	{
		case 0 :
		{
			bullets_num+=0;
		}break;
			case 50:
		{
			bullets_num+=50;
	
		}break;
		case 100 :
		{
		bullets_num+=100;
		
		}break;
				case 150 :
		{

		bullets_num+=150;
		}break;
				case 200 :
		{
		bullets_num+=200;
		}break;
			default:
		{

		}break;
	}	
	}else
	{
		if(shoot_speed_last!=shoot_speed)
		{
		bullets_num--;
		}else
		{
		
		}
	
	}
	shoot_speed_last=shoot_speed;
}
	
	
