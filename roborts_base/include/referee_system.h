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
#ifndef ROBORTS_BASE_REFEREE_SYSTEM_H
#define ROBORTS_BASE_REFEREE_SYSTEM_H

#include "roborts_sdk.h"
#include "ros_dep.h"
#include "module.h"
#include "utils/factory.h"

namespace roborts_base {
/**
 * @brief ROS API for referee system module
 */
class RefereeSystem: public Module {
 public:
  /**
   * @brief Constructor of referee system including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  explicit RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle);
  /**
 * @brief Destructor of referee system
 */
  ~RefereeSystem() = default;
 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();
  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /**  Game Related  **/
  void GameStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_status> raw_game_status);

  void GameResultCallback(const std::shared_ptr<roborts_sdk::cmd_game_result> raw_game_result);

  void GameRobotHPCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_HP> raw_game_survivor);

  void GameEventCallback(const std::shared_ptr<roborts_sdk::cmd_game_event> raw_game_event);

  /**  Robot Related  **/
  void RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_status> raw_robot_status);

  void RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat);

  void RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage);

  void RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot);

  //! ros node handler
  ros::NodeHandle ros_nh_;
  //! ros subscriber
  ros::Publisher ros_game_status_pub_;
  ros::Publisher ros_game_result_pub_ ;
  ros::Publisher ros_game_robot_hp_pub_;
  ros::Publisher ros_game_robot_bullet_pub_;
  ros::Publisher ros_game_zone_array_pub_;

  ros::Publisher ros_robot_status_pub_;
  ros::Publisher ros_robot_heat_pub_;
  ros::Publisher ros_robot_damage_pub_;
  ros::Publisher ros_robot_shoot_pub_;

};
REGISTER_MODULE(Module, "referee_system", RefereeSystem, std::shared_ptr<roborts_sdk::Handle>);
}

#endif //ROBORTS_BASE_REFEREE_SYSTEM_H
