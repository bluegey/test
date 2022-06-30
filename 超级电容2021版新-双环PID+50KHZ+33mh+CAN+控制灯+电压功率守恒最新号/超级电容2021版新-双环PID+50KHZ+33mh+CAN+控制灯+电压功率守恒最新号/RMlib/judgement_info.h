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
/** @file judgement_info.h
 *  @version 1.0
 *  @date June 2017
 *
 *  @brief the information from judgement system
 *
 *  @copyright 2017 DJI RoboMaster. All rights reserved.
 *
 */

#ifndef __JUDGEMENT_INFO_H__
#define __JUDGEMENT_INFO_H__

#include "stm32f10x.h"                  // Device header

#define JUDGE_FRAME_BUFLEN 200

#define RED1_HERO 1
#define RED2_ENGINEER 2
#define RED3_STANDARD 3
#define RED4_STANDARD 4
#define RED5_STANDARD 5
#define RED6_STANDARD 6
#define RED7_SENTINEL 7


#define BLUE1_HERO 11
#define BLUE2_ENGINEER 12
#define BLUE3_STANDARD 13
#define BLUE4_STANDARD 14
#define BLUE5_STANDARD 15
#define BLUE6_STANDARD 16
#define BLUE7_SENTINEL 17


#define RED1_HERO_COM 		0x0101
#define RED2_ENGINEER_COM 0x0102
#define RED3_STANDARD_COM 0x0103
#define RED4_STANDARD_COM 0x0104
#define RED5_STANDARD_COM 0x0105
#define RED6_STANDARD_COM 0x0106
//#define RED7_SENTINEL_COM 0x0107


#define BLUE1_HERO_COM 		 0x0111
#define BLUE2_ENGINEER_COM 0x0112
#define BLUE3_STANDARD_COM 0x0113
#define BLUE4_STANDARD_COM 0x0114
#define BLUE5_STANDARD_COM 0x0115
#define BLUE6_STANDARD_COM 0x0116
//#define BLUE7_SENTINEL_COM 0x0101

/** 
  * @brief  judgement data command id
  */
typedef enum
{
  GAME_STATE_ID       = 0x0001,  //1Hz
  GAME_RESULT_ID = 0x0002,
  ROBORT_HP_DATA_ID = 0x0003,//1Hz
	EVENT_DATA_ID = 0X0101,
  REAL_SUPPLY_DATA_ID  = 0X0102,  //10hZ
  REQUEST_SUPPLY_ID     = 0X0103,//10HZ
  ROBORT_STATE_DATA_ID       = 0X0201,//10
	REAL_HEAT_POWER_DATA_ID				 = 0X0202,//50HZ
  REAL_POSITION_DATA_ID = 0X0203,//10HZ
	REAL_GETBUF_DATA_ID=0X0204,
	REAL_ENERGY_DATA_ID=0X0205,//10HZ
	REAL_HURT_STATE_DATA_ID=0X0206,
	REAL_SHOOT_DATA_ID=0X0207,
	ROBORT_COM_DATA_ID=0X0301,//10HZ
}judge_data_id_e;


/*-------------1. 比赛状态数据： 0x0001。 发送频率： 1Hz---------------------------
字节偏移量 大小 说明
0 1
0-3 bit： 比赛类型
• 1： RoboMaster 机甲大师赛；
• 2： RoboMaster 机甲大师单项赛；
• 3： ICRA RoboMaster 人工智能挑战赛
4-7 bit： 当前比赛阶段
• 0： 未开始比赛；
• 1： 准备阶段；
• 2： 自检阶段；6 © 2019 大疆创新 版权所有
字节偏移量 大小 说明
• 3： 5s 倒计时；
• 4： 对战中；
• 5： 比赛结算中
1 2 当前阶段剩余时间， 单位 s
-------------*/
typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
} ext_game_state_t;

/*-------------2. 比赛结果数据： 0x0002。 发送频率：比赛结束后发送---------
字节偏移量 大小 说明
0 1 0 平局 1 红方胜利 2 蓝方胜利
----*/
typedef __packed struct
{
uint8_t winner;
} ext_game_result_t;

/*-------------3. 机器人存活数据： 0x0003。 发送频率： 1Hz-----
字节偏移量 大小 说明
0 2
bit 0： 红方英雄机器人；
bit 1： 红方工程机器人；
bit 2： 红方步兵机器人 1；
bit 3： 红方步兵机器人 2；
bit 4： 红方步兵机器人 3;
字节偏移量 大小 说明
bit 5： 红方空中机器人；
bit 6： 红方哨兵机器人；
bit 7： 保留
bit 8： 蓝方英雄机器人；
bit 9： 蓝方工程机器人；
bit 10： 蓝方步兵机器人 1；
bit 11： 蓝方步兵机器人 2；
bit 12： 蓝方步兵机器人 3；
bit 13： 蓝方空中机器人；
bit 14： 蓝方哨兵机器人；
bit 15： 保留
对应的 bit 数值置 1 代表机器人存活，数值置 0 代表机器人死亡或者未上场。
--------*/
typedef __packed struct {  
	uint16_t red_1_robot_HP;   
uint16_t red_2_robot_HP;   
uint16_t red_3_robot_HP;   
uint16_t red_4_robot_HP;   
uint16_t red_5_robot_HP;   
uint16_t red_7_robot_HP;   
uint16_t red_base_HP;   
uint16_t blue_1_robot_HP;   
uint16_t blue_2_robot_HP;    
uint16_t blue_3_robot_HP;    
uint16_t blue_4_robot_HP;   
uint16_t blue_5_robot_HP;   
uint16_t blue_7_robot_HP;   
uint16_t blue_base_HP;
} ext_game_robot_HP_t; 


/*-------------4. 场地事件数据： 0x0101。 发送频率：事件改变后发送--------
字节偏移量 大小 说明
0 4
bit 0-1： 己方停机坪占领状态
• 0 为无机器人占领；
• 1 为空中机器人已占领但未停桨；
• 2 为空中机器人已占领并停桨
bit 2： 己方补给站 1 号补血点占领状态 1 为已占领；
字节偏移量 大小 说明
bit 3： 己方补给站 2 号补血点占领状态 1 为已占领；
bit 4： 己方补给站 3 号补血点占领状态 1 为已占领；
bit 5-6： 己方大能量机关状态：
• 0 为打击点未占领且大能量机关未激活；
• 1 为打击点占领且大能量机关未激活；
• 2 为大能量机关已激活；
• 3 为大能量机关已激活且打击点被占领；
bit 7： 己方关口占领状态 1 为已占领；
bit 8： 己方碉堡占领状态 1 为已占领；
bit 9： 己方资源岛占领状态 1 为已占领；
bit 10-11： 己方基地防御状态：
• 2 为基地 100%防御；
• 1 为基地有哨兵防御；
• 0 为基地无防御；
bit 12-13： ICRA 红方防御加成
• 0： 防御加成未激活；
• 1： 防御加成 5s 触发激活中；
• 2： 防御加成已激活
bit 14-15： ICRA 蓝方防御加成
• 0： 防御加成未激活；
• 1： 防御加成 5s 触发激活中；
• 2： 防御加成已激活
其余保留
-----*/
typedef __packed struct
{
uint32_t event_type;
} ext_event_data_t;

/*-------------5. 补给站动作标识： 0x0102。 发送频率：动作改变后发送--------
字节偏移量 大小 说明
0 1
补给站口 ID：
1： 1 号补给口；
2： 2 号补给口
1 1
补弹机器人 ID： 0 为当前无机器人补弹， 1 为红方英雄机器人补弹， 2 为红方工程
机器人补弹， 3/4/5 为红方步兵机器人补弹， 11 为蓝方英雄机器人补弹， 12 为蓝方
工程机器人补弹， 13/14/15 为蓝方步兵机器人补弹
2 1 出弹口开闭状态： 0 为关闭， 1 为子弹准备中， 2 为子弹下落
3 1
补弹数量：
50： 50 颗子弹；
100： 100 颗子弹；
150： 150 颗子弹；
200： 200 颗子弹。
-----*/
typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_projectile_step;
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/*-------------6. 请求补给站补弹子弹： cmd_id (0x0103)。发送频率：上限 10Hz。----------
字节偏移量 大小 说明
0 1
补给站补弹口 ID：
1： 1 号补给口
1 1
补弹机器人 ID： 1 为红方英雄机器人补弹， 2 为红方工程机器人补弹，
3/4/5 为红方步兵机器人补弹， 11 为蓝方英雄机器人补弹， 12 为蓝方
工程机器人补弹， 13/14/15 为蓝方步兵机器人补弹
1 1
补弹数目：
50 ：请求 50 颗子弹下落
---*/

typedef __packed struct
{
uint8_t supply_projectile_id;
uint8_t supply_robot_id;
uint8_t supply_num;
} ext_supply_projectile_booking_t;

/*-------------7. 比赛机器人状态： 0x0201。 发送频率： 10Hz---------
字节偏移量 大小 说明
0 1
机器人 ID：
1： 红方英雄机器人；
2： 红方工程机器人；
3/4/5： 红方步兵机器人；
6： 红方空中机器人；
7： 红方哨兵机器人；
11： 蓝方英雄机器人；
12： 蓝方工程机器人；
13/14/15： 蓝方步兵机器人；
字节偏移量 大小 说明
16： 蓝方空中机器人；
17： 蓝方哨兵机器人。
1 1
机器人等级：
1： 一级； 2： 二级； 3： 三级。
2 2 机器人剩余血量
4 2 机器人上限血量
6 2 机器人 17mm 枪口每秒冷却值
8 2 机器人 17mm 枪口热量上限
10 2 机器人 42mm 枪口每秒冷却值
12 2 机器人 42mm 枪口热量上限
14 1
主控电源输出情况：
0 bit： gimbal 口输出： 1 为有 24V 输出， 0 为无 24v 输出；
1 bit： chassis 口输出： 1 为有 24V 输出， 0 为无 24v 输出；
2 bit： shooter 口输出： 1 为有 24V 输出， 0 为无 24v 输出；
----*/


typedef __packed struct
{
uint8_t robot_id;
uint8_t robot_level;
uint16_t remain_HP;
uint16_t max_HP;
uint16_t shooter_heat0_cooling_rate;
uint16_t shooter_heat0_cooling_limit;
uint16_t shooter_heat1_cooling_rate;
uint16_t shooter_heat1_cooling_limit;
uint8_t mains_power_gimbal_output : 1;
uint8_t mains_power_chassis_output : 1;
uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

/*-------------8. 实时功率热量数据： 0x0202。 发送频率： 50Hz--------
字节偏移量 大小 说明
0 2 底盘输出电压 单位 毫伏
2 2 底盘输出电流 单位 毫安
4 4 底盘输出功率 单位 W 瓦
8 2 底盘功率缓冲 单位 J 焦耳
10 2 17mm 枪口热量
12 2 42mm 枪口热量
-----*/
typedef __packed struct
{
uint16_t chassis_volt;
uint16_t chassis_current;
float chassis_power;
uint16_t chassis_power_buffer;
uint16_t shooter_heat0;
uint16_t shooter_heat1;
} ext_power_heat_data_t;

/*-------------9. 机器人位置： 0x0203。 发送频率： 10Hz---------
字节偏移量 大小 说明
0 4 位置 x 坐标，单位 m
4 4 位置 y 坐标，单位 m
8 4 位置 z 坐标，单位 m
12 4 位置枪口，单位度
----*/
typedef __packed struct
{
float x;
float y;
float z;
float yaw;
} ext_game_robot_pos_t;


/*-------------10. 机器人增益： 0x0204。 发送频率：状态改变后发送-------
字节偏移量 大小 说明
0 1
bit 0： 机器人血量补血状态
bit 1： 枪口热量冷却加速
bit 2： 机器人防御加成
bit 3： 机器人攻击加成
其他 bit 保留
-----*/
typedef __packed struct
{
uint8_t power_rune_buff;
}ext_buff_musk_t;

/*-------------11. 空中机器人能量状态： 0x0205。 发送频率： 10Hz---------
字节偏移量 大小 说明
0 1 积累的能量点
1 2 可攻击时间 单位 s。 50s 递减至 0
----*/
typedef __packed struct
{
uint8_t energy_point;
uint8_t attack_time;
} aerial_robot_energy_t;
/*-------------12. 伤害状态： 0x0206。 发送频率：伤害发生后发送--------
字节偏移量 大小 说明
0 1        bit 0-3： 当血量变化类型为装甲伤害，代表装甲 ID，其中数值为 0-4 号代表机器人-/
---的五个装甲片，其他血量变化类型，该变量数值为 0。
bit 4-7： 血量变化类型
0x0 装甲伤害扣血；
0x1 模块掉线扣血；
0x2 超枪口热量扣血；
0x3 超底盘功率扣血。
-----*/
typedef __packed struct
{
uint8_t armor_id : 4;
uint8_t hurt_type : 4;
} ext_robot_hurt_t;
/*-------------13. 实时射击信息： 0x0207。 发送频率：射击后发送---------
字节偏移量 大小 说明
0 1 子弹类型: 1:17mm 弹丸 2：42mm 弹丸
1 1 子弹射频 单位 Hz
2 4 子弹射速 单位 m/s
----*/
typedef __packed struct
{
uint8_t bullet_type;
uint8_t bullet_freq;
float bullet_speed;
} ext_shoot_data_t;

/*-------------1. 交互数据接收信息： 0x0301。 发送频率：上限 10Hz---------
字节偏移量 大小 说明 备注
0 2 数据段的内容ID
2 2 发送者的ID   需要校验发送者的 ID 正确性，例如红 1 发送给红 5，此项需要校验红 1
4 2 接收者的ID  需要校验接收者的 ID 正确性，例如不能发送到敌对机器人的ID
6 x 内容数据段x 最大为 113
----*/

typedef __packed struct
{
uint16_t data_cmd_id;
uint16_t send_ID;
uint16_t receiver_ID;
}ext_student_interactive_header_data_t;
//图形
typedef __packed struct {
uint8_t operate_tpye;  
uint8_t graphic_tpye; 
uint8_t graphic_name[5]; 
uint8_t layer; 
uint8_t color; 
uint8_t width;  
uint16_t start_x; 
uint16_t start_y; 
uint16_t radius;  
uint16_t end_x;  
uint16_t end_y;  
int16_t start_angle; 
int16_t end_angle; 
uint8_t text_lenght;
uint8_t text[30];
} ext_client_graphic_draw_t;

/** 
  * @brief  student custom data
  */
typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;
  float data1;
  float data2;
  float data3;
	u8    mask;
} client_show_data_t;

typedef __packed struct
{
	ext_student_interactive_header_data_t interactive_data;

ext_client_graphic_draw_t  graphic_draw;
} client_show_graph_t;

typedef __packed struct
{
  uint8_t  data[64];
} user_to_server_t;

typedef __packed struct
{
  uint8_t  data[32];
} server_to_user_t;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{
  ext_game_robot_state_t game_information;
  ext_robot_hurt_t  blood_changed_data;
  ext_shoot_data_t       real_shoot_data;
	ext_power_heat_data_t   real_powerheat_data;
  ext_event_data_t      rfid_data;
	ext_game_robot_HP_t    robot_hp;
//  ext_game_result_t      game_result_data;
  ext_buff_musk_t         get_buff_data;
//	ext_game_robot_pos_t		 gameRobotPos;
	ext_supply_projectile_action_t supply_projectile;
} receive_judge_t;

/* data send (forward) */
/* data receive */
extern receive_judge_t judge_rece_mesg;

void  judgement_data_handler(uint8_t *p_frame);

#endif
