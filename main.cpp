//#include <signal.h>
#include <ros/ros.h>
#include "gcsagent.h"
#include "svo.h"

/*void onSigInt(int sig)
{
    landing = true;
}*/


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gcs_agent"/*, ros::init_options::NoSigintHandler*/);
    //signal(SIGINT, onSigInt);
    ros::NodeHandle nh;
    GCSAgent agent(nh);
    SVO svo(nh, &agent);
    agent.spin();
}
