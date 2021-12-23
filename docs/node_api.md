# Node API

## Parameters

- serial_port(`string`, default: `"/dev/serial_sdk"`)

  Serail port path of usb device. If udev rule correctly configured, the path is mapped to `"/dev/serial_sdk"` as default.

- load_module(`list`, default: `["chassis", "gimbal", "referee_system"]`)

  Loading modules of AI Robot.
  
## Module `chassis`

### Subscribed Topics
- cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

  Chassis velocity command, the chassis is moving at the given speed in the next control cycle.
  
- cmd_vel_acc ([roborts_msgs/TwistAccel](../roborts_msgs/msg/chassis/TwistAccel.msg))

  Chassis velocity and acceleration command, the chassis performs a uniform acceleration motion at an initial given speed in the next control cycle.

### Published Topics

- odom ([nav_msgs/Odometry](http://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html))

  Chassis odometry information with translation calculated from motor encoder and rotation calculated from gyro.

- uwb ([geometry_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html))

  Pose information of the chassis in the UWB coordinate system.

- tf ([tf/tfMessage](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html))

  Publishes the transform from `base_link` to `odom`.
  
## Module `gimbal` 

### Subscribed Topics

- cmd_gimbal_angle ([roborts_msgs/GimbalAngle](../roborts_msgs/msg/gimbal/GimbalAngle.msg))
 
  Gimbal angle command, if `yaw_mode`/`pitch_mode` is set, the `yaw_angle`/`pitch_angle` represents relative control angle.

### Published Topics

- tf ([tf/tfMessage](http://docs.ros.org/api/tf/html/msg/tfMessage.html))

  Publishes the transform from `base_link` to `gimbal`.
  
 
### Services
  
- cmd_fric_wheel ([roborts_msgs/FricWhl](../roborts_msgs/srv/gimbal/FricWhl.srv))

  Start and stop the friction wheel. 
  
  If you want to change the shoot speed(friction wheel motor speed), please change the value of `fric_speed` (normally from 1200 to 1500) in the [gimbal.cpp](../roborts_base/src/gimbal.cpp) and rebuild the package.
  
  ```cpp
      fric_speed.left = 1240;
      fric_speed.right = 1240;
  ```
  The default value is 1240 and shoot speed is about 8 m/s.
  
- cmd_shoot ([roborts_msgs/ShootCmd](../roborts_msgs/srv/gimbal/ShootCmd.srv))

  projectile shoot command with given mode and number.

## Module `referee system`  

### Published Topics

- game_result ([roborts_msgs/GameResult](../roborts_msgs/msg/referee_system/GameResult.msg))

  Game result information:
  
    | Result | Description  | 
    | ------ | :----------: | 
    |  0     | Draw         | 
    |  1     | Red Wins     | 
    |  2     | Blue Wins    | 

- game_robot_bullet ([roborts_msgs/GameRobotBullet](../roborts_msgs/msg/referee_system/GameRobotBullet.msg))

  Game robot remaining bullet information of four robots.

- game_robot_hp ([roborts_msgs/GameRobotHP](../roborts_msgs/msg/referee_system/GameRobotHP.msg))

  Game robot health point information of four robots.

- game_status ([roborts_msgs/GameStatus](../roborts_msgs/msg/referee_system/GameStatus.msg))

  Game status with its remaining time information. 
  
    | Status | Description         | 
    | ------ | :---------------:   | 
    |  0     | Ready               | 
    |  1     | Preparation         | 
    |  2     | Initialize          | 
    |  3     | Five Second CD      | 
    |  4     | Game                | 
    |  5     | End                 | 

- game_zone_array_status ([roborts_msgs/GameZoneArray](../roborts_msgs/msg/referee_system/GameZoneArray.msg))

  Six game zone status with its type and activation flag. The type information is as follow:
  
    | Status | Description         | 
    | ------ | :---------------:   | 
    |  1     | Red HP Recovery     | 
    |  2     | Red Bullet Supply   | 
    |  3     | Blue HP Recovery    | 
    |  4     | Blue Bullet Supply  | 
    |  5     | Disable Shooting    | 
    |  6     | Disable Movement    |   
- lurk_status([roborts_msgs/LurkStatus](../roborts_msgs/msg/referee_system/LurkStatus.msg))

  Indicates whether the current game is in lurking mode.

    | Status | Description         | 
    | ------ | :---------------:   | 
    |  0     | normal		   | 
    |  1     | ready to lurk	   | 
    |  2     | lurking		   | 

- robot_damage ([roborts_msgs/RobotDamage](../roborts_msgs/msg/referee_system/RobotDamage.msg))

  Robot damage type information: 
  
    | Damage Type | Description          | 
    | ----------- | :----------------:   | 
    |  0          | Armor                | 
    |  1          | Offline              | 
    |  2          | Exceed Shoot Speed   | 
    |  3          | Exceed Shoot Heat    | 
    |  4          | Exceed Chassis Power | 
    |  5          | Obstacle Collision   | 
    
  and damage source (armor id) if damage type is armor :
    
    | Damage Source | Description       | 
    | -----------  | :----------------: | 
    |  0           | Forward            | 
    |  1           | Left               | 
    |  2           | Backword           | 
    |  3           | Right              |    

- robot_heat ([roborts_msgs/RobotHeat](../roborts_msgs/msg/referee_system/RobotHeat.msg))

  Chassis voltage, current, power information and shooter heat information
  
- robot_shoot ([roborts_msgs/RobotShoot](../roborts_msgs/msg/referee_system/RobotShoot.msg))

  Robot shoot frequency and speed information

- robot_status ([roborts_msgs/RobotStatus](../roborts_msgs/msg/referee_system/RobotStatus.msg))

  Robot status information including 
  - id, 
  - level, 
  - remain and max health point,
  - heat information
  - gimbal, chassis, shooter power connection status



