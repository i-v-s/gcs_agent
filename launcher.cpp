#include "launcher.h"
#include <mavlink/v1.0/ardupilotmega/mavlink.h>
#include <iostream>
#include <cstdio>
#include <memory>

Launcher::Launcher(ros::NodeHandle & nh, GCSAgent * agent):
    GCSAgentPlugin(agent, MAVLINK_MSG_ID_ROS_CMD_LAUNCH)
{

}

std::string Launcher::exec(const std::string & cmd) {
    std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}

void Launcher::onMessage(const mavlink_message_t & message)
{
    char name[sizeof(mavlink_ros_cmd_launch_t::launch_file) + 1] = {0};
    mavlink_msg_ros_cmd_launch_get_launch_file(&message, name);
    std::string cmd = "roslaunch launch/";
    cmd += name;
    cmd += ".launch";
    exec(cmd);
}
