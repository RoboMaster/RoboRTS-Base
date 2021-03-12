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

#ifndef ROBORTS_BASE_GIMBAL_H
#define ROBORTS_BASE_GIMBAL_H
#include "roborts_sdk.h"
#include "ros_dep.h"
#include "module.h"
#include "utils/factory.h"

namespace roborts_base {
/**
 * @brief ROS API for gimbal module
 */
class Gimbal: public Module {
 public:
  /**
   * @brief Constructor of gimbal including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  explicit Gimbal(std::shared_ptr<roborts_sdk::Handle> handle);
    /**
   * @brief Destructor of gimbal
   */
  ~Gimbal();
 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();
  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /**
   * @brief Gimbal information callback in sdk
   * @param gimbal_info Gimbal information
   */
  void GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info);
  /**
   * @brief Gimbal angle control callback in ROS
   * @param msg Gimbal angle control data
   */
  void GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg);
  /**
   * @brief Control friction wheel service callback in ROS
   * @param req Friction wheel control data as request
   * @param res Control result as response
   * @return True if success
   */
  bool CtrlFricWheelService(roborts_msgs::FricWhl::Request &req,
                            roborts_msgs::FricWhl::Response &res);
  /**
   * @brief Control shoot service callback in ROS
   * @param req Shoot control data as request
   * @param res Control result as response
   * @return True if success
   */
  bool CtrlShootService(roborts_msgs::ShootCmd::Request &req,
                        roborts_msgs::ShootCmd::Response &res);

  //! sdk version client
  std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                      roborts_sdk::cmd_version_id>> verison_client_;

  //! sdk heartbeat thread
  std::thread heartbeat_thread_;
  //! sdk publisher for heartbeat
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;


  //! sdk publisher for gimbal angle control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_gimbal_angle>>     gimbal_angle_pub_;
  //! sdk publisher for frcition wheel control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_fric_wheel_speed>> fric_wheel_pub_;
  //! sdk publisher for gimbal shoot control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_shoot_info>>       gimbal_shoot_pub_;

  //! ros node handler
  ros::NodeHandle    ros_nh_;
  //! ros subscriber for gimbal angle control
  ros::Subscriber    ros_sub_cmd_gimbal_angle_;
  //! ros service server for friction wheel control
  ros::ServiceServer ros_ctrl_fric_wheel_srv_;
  //! ros service server for gimbal shoot control
  ros::ServiceServer ros_ctrl_shoot_srv_;
  //! ros gimbal tf
  geometry_msgs::TransformStamped gimbal_tf_;
  //! ros gimbal tf broadcaster
  tf::TransformBroadcaster        tf_broadcaster_;

};
REGISTER_MODULE(Module, "gimbal", Gimbal, std::shared_ptr<roborts_sdk::Handle>);
}
#endif //ROBORTS_BASE_GIMBAL_H
