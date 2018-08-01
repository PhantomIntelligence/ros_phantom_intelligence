cmake_minimum_required(VERSION 2.8.2 FATAL_ERROR)
project(phantom_intelligence_msgs)

find_package(catkin REQUIRED
        COMPONENTS
        message_generation
        std_msgs
        )

add_message_files(
        DIRECTORY msg
        FILES
        PhantomAWL.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
