/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
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

#include "referee_system.h"

namespace roborts_base {
RefereeSystem::RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle) :
    Module(handle) {
  SDK_Init();
  ROS_Init();
}
void RefereeSystem::SDK_Init() {
  /**  Game Related  **/
  handle_->CreateSubscriber<roborts_sdk::cmd_game_status>(REFEREE_GAME_CMD_SET, CMD_GAME_STATUS,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::GameStatusCallback,
                                                                   this,
                                                                   std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_game_result>(REFEREE_GAME_CMD_SET, CMD_GAME_RESULT,
                                                          CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                          std::bind(&RefereeSystem::GameResultCallback,
                                                                    this,
                                                                    std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_game_robot_HP>(REFEREE_GAME_CMD_SET, CMD_GAME_ROBOT_HP,
                                                                   CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                                   std::bind(&RefereeSystem::GameRobotHPCallback,
                                                                             this,
                                                                             std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_game_event>(REFEREE_GAME_CMD_SET, CMD_GAME_EVENT,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::GameEventCallback,
                                                                   this,
                                                                   std::placeholders::_1));

  /** Robot Related **/
  handle_->CreateSubscriber<roborts_sdk::cmd_game_robot_status>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_STATUS,
                                                               CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                               std::bind(&RefereeSystem::RobotStatusCallback,
                                                                         this,
                                                                         std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_power_heat_data>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_POWER_HEAT,
                                                              CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                              std::bind(&RefereeSystem::RobotHeatCallback,
                                                                        this,
                                                                        std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_robot_hurt>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_HURT,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::RobotDamageCallback,
                                                                   this,
                                                                   std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_shoot_data>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_SHOOT,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::RobotShootCallback,
                                                                   this,
                                                                   std::placeholders::_1));

}
void RefereeSystem::ROS_Init() {
  //ros publisher
  /**  Game Related  **/
  ros_game_status_pub_ = ros_nh_.advertise<roborts_msgs::GameStatus>("game_status", 30);
  ros_game_result_pub_ = ros_nh_.advertise<roborts_msgs::GameResult>("game_result", 30);
  ros_game_robot_hp_pub_ = ros_nh_.advertise<roborts_msgs::GameRobotHP>("game_robot_hp", 30);
  ros_game_robot_bullet_pub_ = ros_nh_.advertise<roborts_msgs::GameRobotBullet>("game_robot_bullet", 30);
  ros_game_zone_array_pub_ = ros_nh_.advertise<roborts_msgs::GameZoneArray>("game_zone_array_status", 30);
  ros_game_lurk_status_pub_=ros_nh_.advertise<roborts_msgs::LurkStatus>("game_lurk_status",30);
  /** Robot Related **/
  ros_robot_status_pub_ = ros_nh_.advertise<roborts_msgs::RobotStatus>("robot_status", 30);
  ros_robot_heat_pub_ = ros_nh_.advertise<roborts_msgs::RobotHeat>("robot_heat", 30);
  ros_robot_damage_pub_ = ros_nh_.advertise<roborts_msgs::RobotDamage>("robot_damage", 30);
  ros_robot_shoot_pub_ = ros_nh_.advertise<roborts_msgs::RobotShoot>("robot_shoot", 30);
}

void RefereeSystem::GameStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_status> raw_game_status){
  roborts_msgs::GameStatus game_status;
  ROS_ASSERT(raw_game_status->game_type == 3);
  game_status.game_status    = raw_game_status->game_progress;
  game_status.remaining_time = raw_game_status->stage_remain_time;
  ros_game_status_pub_.publish(game_status);
}

void RefereeSystem::GameResultCallback(const std::shared_ptr<roborts_sdk::cmd_game_result> raw_game_result){
  roborts_msgs::GameResult game_result;
  game_result.result = raw_game_result->winner;
  ros_game_result_pub_.publish(game_result);
}

void RefereeSystem::GameRobotHPCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_HP> raw_game_robot_hp){
  roborts_msgs::GameRobotHP game_robot_hp;
  game_robot_hp.red1  = raw_game_robot_hp->red_1_robot_HP;
  game_robot_hp.red2  = raw_game_robot_hp->red_2_robot_HP;
  game_robot_hp.blue1 = raw_game_robot_hp->blue_1_robot_HP;
  game_robot_hp.blue2 = raw_game_robot_hp->blue_2_robot_HP;
  ros_game_robot_hp_pub_.publish(game_robot_hp);
}

void RefereeSystem::GameEventCallback(const std::shared_ptr<roborts_sdk::cmd_game_event> raw_game_event){
  roborts_msgs::GameRobotBullet game_robot_bullet;
  game_robot_bullet.red1  = raw_game_event->red1_bullet_left;
  game_robot_bullet.red2  = raw_game_event->red2_bullet_left;
  game_robot_bullet.blue1 = raw_game_event->blue1_bullet_left;
  game_robot_bullet.blue2 = raw_game_event->blue2_bullet_left;
  ros_game_robot_bullet_pub_.publish(game_robot_bullet);

  roborts_msgs::GameZoneArray game_zone_array;
  game_zone_array.zone[0].type   = raw_game_event->F1_zone_buff_debuff_status;
  game_zone_array.zone[0].active = bool(raw_game_event->F1_zone_status);
  game_zone_array.zone[1].type   = raw_game_event->F2_zone_buff_debuff_status;
  game_zone_array.zone[1].active = bool(raw_game_event->F2_zone_status);
  game_zone_array.zone[2].type   = raw_game_event->F3_zone_buff_debuff_status;
  game_zone_array.zone[2].active = bool(raw_game_event->F3_zone_status);
  game_zone_array.zone[3].type   = raw_game_event->F4_zone_buff_debuff_status;
  game_zone_array.zone[3].active = bool(raw_game_event->F4_zone_status);
  game_zone_array.zone[4].type   = raw_game_event->F5_zone_buff_debuff_status;
  game_zone_array.zone[4].active = bool(raw_game_event->F5_zone_status);
  game_zone_array.zone[5].type   = raw_game_event->F6_zone_buff_debuff_status;
  game_zone_array.zone[5].active = bool(raw_game_event->F6_zone_status);
  
  ros_game_zone_array_pub_.publish(game_zone_array);

roborts_msgs::LurkStatus game_lurk_status;
game_lurk_status.lurk_mode = raw_game_event->lurk_mode;
ros_game_lurk_status_pub_.publish(game_lurk_status);

}

void RefereeSystem::RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_status> raw_robot_status){
  roborts_msgs::RobotStatus robot_status;

  if ( raw_robot_status->robot_id != 1 && raw_robot_status->robot_id != 2 && 
       raw_robot_status->robot_id != 101 && raw_robot_status->robot_id != 102 ){

    ROS_WARN("For AI challenge, "
             "please set robot id to Blue1/2 or Red1/2 in the referee system main control module, "
             "currently the id is %u", robot_status.id);
  }  
  robot_status.id = raw_robot_status->robot_id;
  robot_status.level = raw_robot_status->robot_level;
  robot_status.remain_hp = raw_robot_status->remain_HP;
  robot_status.max_hp = raw_robot_status->max_HP;

  robot_status.heat_cooling_limit = raw_robot_status->shooter_id1_17mm_cooling_limit;
  robot_status.heat_cooling_rate = raw_robot_status->shooter_id1_17mm_cooling_rate;
  robot_status.heat_speed_limit = raw_robot_status->shooter_id1_17mm_speed_limit;

  robot_status.chassis_enable = bool(raw_robot_status->mains_power_chassis_output);
  robot_status.gimbal_enable = bool(raw_robot_status->mains_power_gimbal_output);
  robot_status.shooter_enable = bool(raw_robot_status->mains_power_shooter_output);
  ros_robot_status_pub_.publish(robot_status);
}

void RefereeSystem::RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat){
  roborts_msgs::RobotHeat robot_heat;
  robot_heat.chassis_volt = raw_robot_heat->chassis_volt;
  robot_heat.chassis_current = raw_robot_heat->chassis_current;
  robot_heat.chassis_power = raw_robot_heat->chassis_power;
  robot_heat.chassis_power_buffer = raw_robot_heat->chassis_power_buffer;
  robot_heat.shooter_heat = raw_robot_heat->shooter_id1_17mm_cooling_heat;
  ros_robot_heat_pub_.publish(robot_heat);
}

void RefereeSystem::RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage){
  roborts_msgs::RobotDamage robot_damage;
  robot_damage.damage_type = raw_robot_damage->hurt_type;
  robot_damage.damage_source = raw_robot_damage->armor_id;
  ros_robot_damage_pub_.publish(robot_damage);
}

void RefereeSystem::RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot){
  roborts_msgs::RobotShoot robot_shoot;
  ROS_ASSERT(raw_robot_shoot->bullet_type == 1);
  ROS_ASSERT(raw_robot_shoot->shooter_id == 1);
  robot_shoot.frequency = raw_robot_shoot->bullet_freq;
  robot_shoot.speed = raw_robot_shoot->bullet_speed;
  ros_robot_shoot_pub_.publish(robot_shoot);
}
}
