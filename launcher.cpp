#include "launcher.h"
#include <mavlink/v1.0/ardupilotmega/mavlink.h>
//#include <iostream>
//#include <cstdio>
#include <unistd.h>
#include <signal.h>
#include <memory>

Launcher::Launcher(ros::NodeHandle & nh, GCSAgent * agent):
    GCSAgentPlugin(agent, MAVLINK_MSG_ID_ROS_CMD_LAUNCH), _pid(0)
{
    nh.param<std::string>("launch_path", launchPath, "~/viscopter/launch");
    launchPath += "/";
}

void Launcher::launch(const std::string & launchFile)
{
    if(_pid)
    {
        ROS_INFO("Kill process");
        kill(_pid, SIGTERM);
        _pid = 0;
    }
    if(launchFile == "") return;
    pid_t pid = fork();
    if(pid == 0)
    {
        //const char * argv[] = {"/bin/sh", "-c", ("roslaunch " + launchFile).c_str(), 0};
        const char * argv[] = {"/opt/ros/jade/bin/roslaunch", launchFile.c_str(), 0};
        if(execv(argv[0], (char **)argv) == -1)
        {
            perror("child process execve failed [%m]");
            ROS_ERROR("gcs_agent::Launcher: Unable to execve (errno: %d)", errno);
        }
        /*if(system(("roslaunch " + launchFile).c_str()) == -1)
            ROS_ERROR("gcs_agent::Launcher: Unable to exec system() (errno: %d)", errno);*/
        _exit(EXIT_SUCCESS);
    }
    if(pid == -1)
    {
        ROS_ERROR("gcs_agent::Launcher: Unable to fork (errno: %d)", errno);
        return;
    }

    ROS_INFO("PID: %d", pid);
    _pid = pid;
    /*std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;*/
}

void Launcher::onMessage(const mavlink_message_t & message)
{
    char name[sizeof(mavlink_ros_cmd_launch_t::launch_file) + 1] = {0};
    mavlink_msg_ros_cmd_launch_get_launch_file(&message, name);
    if(!strcmp(name, "[stop]"))
        launch("");
    else
    {
        std::string launchFile = launchPath;
        launchFile += name;
        launchFile += ".launch";
        launch(launchFile);
    }
}
