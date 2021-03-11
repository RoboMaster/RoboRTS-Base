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

#include "roborts_sdk.h"
#include "config.h"
#include "module.h"
#include "utils/factory.h"

using SDKModuleFactory=ModuleFactory<roborts_base::Module,
std::shared_ptr<roborts_sdk::Handle>>;
int main(int argc, char **argv){
  GLogWrapper glog_wrapper(argv[0]);
  ros::init(argc, argv, "roborts_base_node");
  ros::NodeHandle nh;
  // init config from ros parameter
  roborts_base::Config config;
  config.GetParam();
  // create sdk handler
  auto handle = std::make_shared<roborts_sdk::Handle>(config.serial_port);
  if(!handle->Init()) return 1;
  // list registed module
  auto registed_module = SDKModuleFactory::GetModuleName();
  for(auto registed_module_name: registed_module){
    ROS_INFO_STREAM("Module "
                        <<CL_BOLDGREEN<<registed_module_name<<CL_RESET
                        <<" has been registed.");
  }
  // load modules according to configuration
  std::unordered_map<std::string, std::unique_ptr<roborts_base::Module>> load_module_dict;
  for(auto load_module_name: config.load_module){
    auto module = ModuleFactory<roborts_base::Module,
    std::shared_ptr<roborts_sdk::Handle>>::CreateModule(load_module_name, handle);
    if(module==nullptr){
      ROS_WARN_STREAM("Module "
                          <<CL_BOLDRED<<load_module_name<<CL_YELLOW
                          <<" can not be loaded as you haven't register it.");
      continue;
    }
    load_module_dict[load_module_name] = std::move(module);
    ROS_INFO_STREAM("Module "
                        <<CL_BOLDBLUE<<load_module_name<<CL_RESET
                        <<" has been loaded.");
  }
  // spin
  while(ros::ok()) {
    handle->Spin();
    ros::spinOnce();
    usleep(1000);
  }

}