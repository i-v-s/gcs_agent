#ifndef LAUNCHER_H
#define LAUNCHER_H
#include <ros/ros.h>
#include "gcsagent.h"
#include <string>

class Launcher: public GCSAgentPlugin
{
    void launch(const std::string & cmd);
    std::string launchPath;
    virtual void onMessage(const mavlink_message_t & message);
    pid_t _pid;
public:
    virtual ~Launcher() {}
    Launcher(ros::NodeHandle & nh, GCSAgent * agent);

};


#endif // LAUNCHER_H
