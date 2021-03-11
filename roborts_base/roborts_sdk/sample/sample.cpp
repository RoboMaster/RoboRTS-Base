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

#include <iostream>
#include <atomic>
#include "signal.h"
#include "roborts_sdk.h"
std::atomic<bool> ok;
void SignalHandler(int signal){
  LOG_INFO<<"Shutdown the program.";
  ok = false;
}

int main(int argc, char **argv) {
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  GLogWrapper glog_wrapper(argv[0]);
  std::string serial_path = "/dev/ttyACM0";
  auto sdk_handle=std::make_shared<roborts_sdk::Handle>(serial_path);
  if(!sdk_handle->Init()) return 1;
  /*-----------Subscriber Test-------------*/
  int count;

  auto func = [&count] (const std::shared_ptr<roborts_sdk::cmd_chassis_info> message) -> void{
    LOG_INFO<<"chassis_msg_"<<count<<" position_x_mm: "<<(int)(message->position_x_mm);
    count++;
  };
  auto func2 = [&count] (const std::shared_ptr<roborts_sdk::cmd_gimbal_info> message) -> void{
    LOG_INFO<<"gimbal_msg_"<<count<<" pitch_gyro_angle: "<<(int)message->pitch_gyro_angle;
    count++;
  };
  auto func3 = [&count] (const std::shared_ptr<roborts_sdk::cmd_uwb_info> message) -> void{
    LOG_INFO<<"uwb_msg_"<<count<<" x: "<<(int)message->x;
    count++;
  };
  auto sub1=sdk_handle->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
                                                           CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,func);
  auto sub2=sdk_handle->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET, CMD_PUSH_GIMBAL_INFO,
                                                          GIMBAL_ADDRESS, BROADCAST_ADDRESS,func2);
  auto sub3=sdk_handle->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
                                                       CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,func3);

  /*-----------Publisher Test-------------*/
  auto pub1 = sdk_handle->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET,CMD_SET_CHASSIS_SPEED,
      MANIFOLD2_ADDRESS,CHASSIS_ADDRESS);
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.rotate_x_offset=0;
  chassis_speed.rotate_y_offset=0;
  chassis_speed.vx=100;
  chassis_speed.vy=0;
  chassis_speed.vw=0;
  pub1->Publish(chassis_speed);

  /*-----------Client Test-------------*/
  auto client1 = sdk_handle->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>(UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
                                                                         MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  auto func4 = [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                      roborts_sdk::cmd_version_id>::SharedFuture future) {
    char version_str[30];
    snprintf(version_str, sizeof(version_str), "%d.%d.%d.%d",
             int(future.get()->version_id>>24&0xFF),
             int(future.get()->version_id>>16&0xFF),
             int(future.get()->version_id>>8&0xFF),
             int(future.get()->version_id&0xFF));
    LOG_INFO<<"chassis firmware version: "<< version_str;
  };
  client1->AsyncSendRequest(version, func4);
  ok = true;

  while(ok){
    sdk_handle->Spin();
    usleep(10);
  }
}
