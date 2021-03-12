/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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
#ifndef ROBORTS_SDK_PROTOCOL_CONTENT_H
#define ROBORTS_SDK_PROTOCOL_CONTENT_H
#include <stdint.h>
namespace roborts_sdk {

#pragma pack(push, 1)
//DEVICE_ADDRESS
#define MANIFOLD2_ADDRESS              (0x00u)
#define CHASSIS_ADDRESS                (0X01u)
#define GIMBAL_ADDRESS                 (0X02u)
#define BROADCAST_ADDRESS              (0Xffu)

//CMD_SET                            
#define UNIVERSAL_CMD_SET              (0x00u)
#define REFEREE_SEND_CMD_SET           (0x01u)
#define CHASSIS_CMD_SET                (0x02u)
#define GIMBAL_CMD_SET                 (0x03u)
#define COMPATIBLE_CMD_SET             (0x04u)


#define REFEREE_GAME_CMD_SET           (0x40u)
#define REFEREE_BATTLEFIELD_CMD_SET    (0x41u)
#define REFEREE_ROBOT_CMD_SET          (0x42u)
#define REFEREE_RECEIVE_CMD_SET        (0x43u)

#define TEST_CMD_SET                   (0xFFu)

/*----------------------------UNIVERSAL_CMD_SET--- 0x00 ---------------------*/
/*
 *  cmd_set:  UNIVERSAL_CMD_SET
 *  cmd_id:   CMD_HEARTBEAT
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: CHASSIS_ADDRESS/GIMBAL_ADDRESS
 *  msg_type: cmd_heartbeat
 *  session:  no ack
 */
#define CMD_HEARTBEAT                  (0x01u)
typedef struct{
  uint32_t heartbeat;
} cmd_heartbeat;

/*
 *  cmd_set:  UNIVERSAL_CMD_SET
 *  cmd_id:   CMD_REPORT_VERSION
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: CHASSIS_ADDRESS/GIMBAL_ADDRESS
 *  msg_type: cmd_version_id
 *  session:  need ack
 *  ack_type: cmd_version_id
 */
#define CMD_REPORT_VERSION             (0X02u)
typedef struct
{
  uint32_t version_id;
} cmd_version_id;

/*-----------------------------REFEREE_SEND_CMD--- 0x01 ---------------------*/
//// TODO: Deprecated
//#define CMD_REFEREE_SEND_DATA               (0X01u)
//typedef struct
//{
//  uint16_t cmd = 0x0103u;
//  uint8_t supply_projectile_id;
//  uint8_t supply_robot_id;
//  uint8_t supply_num;
//} cmd_supply_projectile_booking;

// TODO: NOT IMPLEMENT
/*--------------------------REFEREE_SEND_CMD----0x01-------------------------*/
/*--------------------------REFEREE_RECV_CMD--- 0x43-------------------------*/
/*  Define your own protocol for comm between different robots here*/
/*    typedef __packed struct
/*    {
/*        uint16_t cmd;
/*        uint16_t sender_robot_id;
/*        uint16_t receiver_robot_id;
/*        CertainType data;
/*    } ext_robot_comm_t;
 */

/*-----------------------------CHASSIS_CMD_SET---- 0x02 ---------------------*/
/*
 *  cmd_set:  CHASSIS_CMD_SET
 *  cmd_id:   CMD_PUSH_CHASSIS_INFO
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_chassis_info
 *  session:  no ack
 */
#define CMD_PUSH_CHASSIS_INFO          (0X01u)
typedef struct {
  int16_t gyro_angle;
  int16_t gyro_rate;
  int32_t position_x_mm;
  int32_t position_y_mm;
  int16_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
} cmd_chassis_info;

/*
 *  cmd_set:  CHASSIS_CMD_SET
 *  cmd_id:   CMD_SET_CHASSIS_SPEED
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: CHASSIS_ADDRESS
 *  msg_type: cmd_chassis_speed
 *  session:  no ack
 */
#define CMD_SET_CHASSIS_SPEED          (0X03u)
typedef struct {
  int16_t vx;
  int16_t vy;
  int16_t vw;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
} cmd_chassis_speed;

/*
 *  cmd_set:  CHASSIS_CMD_SET
 *  cmd_id:   CMD_GET_CHASSIS_PARAM
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_chassis_param
 *  session:  no ack
 */
// TODO NOT IMPLEMENT
#define CMD_GET_CHASSIS_PARAM          (0X04u)
typedef struct {
  uint16_t wheel_perimeter;
  uint16_t wheel_track;
  uint16_t wheel_base;
  int16_t gimbal_x_offset;
  int16_t gimbal_y_offset;
} cmd_chassis_param;

/*
 *  cmd_set:  CHASSIS_CMD_SET
 *  cmd_id:   CMD_SET_CHASSIS_SPD_ACC
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: CHASSIS_ADDRESS
 *  msg_type: cmd_chassis_spd_acc
 *  session:  no ack
 */
#define CMD_SET_CHASSIS_SPD_ACC        (0X05u)
typedef struct {
  int16_t vx;
  int16_t vy;
  int16_t vw;
  int16_t ax;
  int16_t ay;
  int16_t wz;
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
} cmd_chassis_spd_acc;

/*-----------------------------GIMBAL_CMD_SET---- 0x03 ---------------------*/
/*
 *  cmd_set:  GIMBAL_CMD_SET
 *  cmd_id:   CMD_PUSH_GIMBAL_INFO
 *  sender:   GIMBAL_ADDRESS
 *  receiver: BROADCAST_ADDRESS
 *  msg_type: cmd_gimbal_info
 *  session:  no ack
 */
#define CMD_PUSH_GIMBAL_INFO           (0X01u)
typedef struct {
  uint8_t mode;
  int16_t pitch_ecd_angle;
  int16_t yaw_ecd_angle;
  int16_t pitch_gyro_angle;
  int16_t yaw_gyro_angle;
  int16_t yaw_rate;
  int16_t pitch_rate;
} cmd_gimbal_info;

/*
 *  cmd_set:  GIMBAL_CMD_SET
 *  cmd_id:   CMD_PUSH_GIMBAL_INFO
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: GIMBAL_ADDRESS
 *  msg_type: gimbal_mode_e
 *  session:  no ack
 */
// Deprecated Command
//#define CMD_SET_GIMBAL_MODE            (0X02u)
//typedef enum: uint8_t {
//  CODE_CONTROL,
//  GYRO_CONTROL,
//  G_MODE_MAX_NUM,
//} gimbal_mode_e;

/*
 *  cmd_set:  GIMBAL_CMD_SET
 *  cmd_id:   CMD_SET_GIMBAL_ANGLE
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: GIMBAL_ADDRESS
 *  msg_type: cmd_gimbal_angle
 *  session:  no ack
 */
#define CMD_SET_GIMBAL_ANGLE           (0x03u)
typedef struct{
  union{
    uint8_t flag;
    struct {
      uint8_t yaw_mode:   1;//0 means absolute, 1 means relative;
      uint8_t pitch_mode: 1;
    } bit;
  } ctrl;
  int16_t pitch;
  int16_t yaw;
}cmd_gimbal_angle;


/*
 *  cmd_set:  GIMBAL_CMD_SET
 *  cmd_id:   CMD_SET_FRIC_WHEEL_SPEED
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: GIMBAL_ADDRESS
 *  msg_type: cmd_fric_wheel_speed
 *  session:  no ack
 */
#define CMD_SET_FRIC_WHEEL_SPEED       (0X04u)
typedef struct{
  uint16_t left;
  uint16_t right;
} cmd_fric_wheel_speed;

/*
 *  cmd_set:  GIMBAL_CMD_SET
 *  cmd_id:   CMD_SET_SHOOT_INFO
 *  sender:   MANIFOLD2_ADDRESS
 *  receiver: GIMBAL_ADDRESS
 *  msg_type: cmd_shoot_info
 *  session:  no ack
 */
#define CMD_SET_SHOOT_INFO             (0x05u)
typedef enum: uint8_t {
  SHOOT_STOP = 0,
  SHOOT_ONCE,
  SHOOT_CONTINUOUS,
} shoot_cmd_e;

typedef struct {
  uint8_t shoot_cmd; // shoot_cmd_e
  uint32_t shoot_add_num;
  uint16_t shoot_freq;
} cmd_shoot_info;

/*------------------------COMPATIBLE_CMD_SET---- 0x04 -------------------*/
#define CMD_RC_DATA_FORWARD            (0X01u)

/*
 *  cmd_set:  COMPATIBLE_CMD_SET
 *  cmd_id:   CMD_PUSH_UWB_INFO
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_uwb_info
 *  session:  no ack
 */
#define CMD_PUSH_UWB_INFO              (0X02u)
typedef struct {
  int16_t x;
  int16_t y;
  uint16_t yaw;
  int16_t distance[6];
  uint16_t error;
  uint16_t res;
} cmd_uwb_info;

/*------------------------REFEREE_GAME_CMD_SET---- 0x40 -------------------*/
/*
 *  cmd_set:  REFEREE_GAME_CMD_SET
 *  cmd_id:   CMD_GAME_STATUS
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_game_state
 *  session:  no ack
 */
#define CMD_GAME_STATUS            (0X01u)
typedef struct
{
  uint8_t game_type : 4;
  uint8_t game_progress : 4;
  uint16_t stage_remain_time;
  uint64_t sync_timestamp;
} cmd_game_status;

/*
 *  cmd_set:  REFEREE_GAME_CMD_SET
 *  cmd_id:   CMD_GAME_RESULT
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_game_result
 *  session:  no ack
 */
#define CMD_GAME_RESULT            (0X02u)
typedef struct
{
  uint8_t winner;
} cmd_game_result;

/*
 *  cmd_set:  REFEREE_GAME_CMD_SET
 *  cmd_id:   CMD_GAME_ROBOT_HP
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_game_robot_HP
 *  session:  no ack
 */
#define CMD_GAME_ROBOT_HP          (0X03u)
typedef struct
{
  uint16_t red_1_robot_HP;
  uint16_t red_2_robot_HP;
  uint16_t red_3_robot_HP;
  uint16_t red_4_robot_HP;
  uint16_t red_5_robot_HP;
  uint16_t red_7_robot_HP;
  uint16_t red_outpost_HP;
  uint16_t red_base_HP;

  uint16_t blue_1_robot_HP;
  uint16_t blue_2_robot_HP;
  uint16_t blue_3_robot_HP;
  uint16_t blue_4_robot_HP;
  uint16_t blue_5_robot_HP;
  uint16_t blue_7_robot_HP;
  uint16_t blue_outpost_HP;
  uint16_t blue_base_HP;
} cmd_game_robot_HP;


/*
 *  cmd_set:  REFEREE_GAME_CMD_SET
 *  cmd_id:   CMD_GAME_EVENT
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_game_event
 *  session:  no ack
 */
#define CMD_GAME_EVENT          (0X05u)
typedef struct
{
  uint8_t F1_zone_status: 1;
  uint8_t F1_zone_buff_debuff_status: 3;
  uint8_t F2_zone_status: 1;
  uint8_t F2_zone_buff_debuff_status: 3;
  uint8_t F3_zone_status: 1;
  uint8_t F3_zone_buff_debuff_status: 3;
  uint8_t F4_zone_status: 1;
  uint8_t F4_zone_buff_debuff_status: 3;
  uint8_t F5_zone_status: 1;
  uint8_t F5_zone_buff_debuff_status: 3;
  uint8_t F6_zone_status: 1;
  uint8_t F6_zone_buff_debuff_status: 3;

  uint16_t red1_bullet_left;
  uint16_t red2_bullet_left;
  uint16_t blue1_bullet_left;
  uint16_t blue2_bullet_left;
} cmd_game_event;

/*-------------------REFEREE_BATTLEFIELD_CMD_SET---- 0x41 -------------*/


/*------------------------REFEREE_ROBOT_CMD---- 0x42 -------------------*/
/*
 *  cmd_set:  REFEREE_ROBOT_CMD
 *  cmd_id:   CMD_ROBOT_STATUS
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_game_robot_status
 *  session:  no ack
 */
#define CMD_ROBOT_STATUS           (0X01u)
typedef struct
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t remain_HP;
  uint16_t max_HP;

  uint16_t shooter_id1_17mm_cooling_rate;
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;

  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
  uint16_t shooter_id2_17mm_speed_limit;

  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;

  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} cmd_game_robot_status;

/*
 *  cmd_set:  REFEREE_ROBOT_CMD
 *  cmd_id:   CMD_ROBOT_POWER_HEAT
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_power_heat_data
 *  session:  no ack
 */
#define CMD_ROBOT_POWER_HEAT         (0X02u)
typedef struct
{
  uint16_t chassis_volt;
  uint16_t chassis_current;
  float chassis_power;
  uint16_t chassis_power_buffer;
  uint16_t shooter_id1_17mm_cooling_heat;
  uint16_t shooter_id2_17mm_cooling_heat;
  uint16_t shooter_id1_42mm_cooling_heat;
} cmd_power_heat_data;

/*
 *  cmd_set:  REFEREE_ROBOT_CMD
 *  cmd_id:   CMD_ROBOT_HURT
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_robot_hurt
 *  session:  no ack
 */
#define CMD_ROBOT_HURT                (0X06u)
typedef struct
{
  uint8_t armor_id : 4;
  uint8_t hurt_type : 4;
} cmd_robot_hurt;

/*
 *  cmd_set:  REFEREE_ROBOT_CMD
 *  cmd_id:   CMD_ROBOT_SHOOT
 *  sender:   CHASSIS_ADDRESS
 *  receiver: MANIFOLD2_ADDRESS
 *  msg_type: cmd_shoot_data
 *  session:  no ack
 */
#define CMD_ROBOT_SHOOT               (0X07u)
typedef struct
{
  uint8_t bullet_type;
  uint8_t shooter_id;
  uint8_t bullet_freq;
  float bullet_speed;
} cmd_shoot_data;

/*-----------------------------TEST_CMD---- 0xFF ---------------------*/
#define TEXT_ECHO_TRANSMIT             (0x00u)
#pragma pack(pop)
}
#endif //ROBORTS_SDK_PROTOCOL_CONTENT_H
