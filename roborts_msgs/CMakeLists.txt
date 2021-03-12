project(roborts_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})
find_package(catkin REQUIRED COMPONENTS
        std_msgs
        nav_msgs
        geometry_msgs
        message_generation
        )

#chassis message
add_message_files(
        DIRECTORY msg/chassis
        FILES
        TwistAccel.msg
)

# gimbal message
add_message_files(
        DIRECTORY msg/gimbal
        FILES
        GimbalAngle.msg
)

# referee_system message
add_message_files(
        DIRECTORY msg/referee_system
        FILES
        GameStatus.msg
        GameResult.msg
        GameRobotHP.msg
        GameRobotBullet.msg
        GameZone.msg
        GameZoneArray.msg
        RobotStatus.msg
        RobotHeat.msg
        RobotDamage.msg
        RobotShoot.msg
)

# gimbal message
add_service_files(
        DIRECTORY srv/gimbal
        FILES
        FricWhl.srv
        ShootCmd.srv
)

# dependencies
generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs nav_msgs message_runtime)