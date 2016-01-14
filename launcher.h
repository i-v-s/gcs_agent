#ifndef LAUNCHER_H
#define LAUNCHER_H
#include <ros/ros.h>
#include "gcsagent.h"
#include <string>

class Launcher: public GCSAgentPlugin
{
    std::string exec(const std::string & cmd);
    virtual void onMessage(const mavlink_message_t & message);
public:
    virtual ~Launcher() {}
    Launcher(ros::NodeHandle & nh, GCSAgent * agent);

};


#endif // LAUNCHER_H
