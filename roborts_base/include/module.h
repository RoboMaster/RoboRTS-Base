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

#ifndef ROBORTS_BASE_MODULE_H
#define ROBORTS_BASE_MODULE_H

#include <memory>
#include "roborts_sdk.h"

namespace roborts_base {
/**
 * @brief ROS API for module base
 */
class Module {
 public:
  /**
   * @brief Constructor of module
   * @param handle handler of sdk
   */
  Module(std::shared_ptr<roborts_sdk::Handle> handle):handle_(handle){}
  /**
     * @brief Deconstructor of module
   */
  virtual ~Module() = default;

 protected:
  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;
};
}
#endif //ROBORTS_BASE_MODULE_H
