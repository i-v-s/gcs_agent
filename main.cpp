#include <ros/ros.h>
#include "gcsagent.h"
#include "svo.h"
#include "launcher.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_agent");
    ros::NodeHandle nh;
    GCSAgent agent(nh);
    SVO svo(nh, &agent);
    Launcher launcher(nh, &agent);
    agent.spin();
}
