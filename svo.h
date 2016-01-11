#include <ros/ros.h>
#include <svo_msgs/Info.h>
#include "gcsagent.h"
//#include "msf.h"

class SVO: public GCSAgentPlugin
{
protected:
	ros::Subscriber subInfo;
	ros::Publisher pubKey;
	void onInfo(const svo_msgs::Info::ConstPtr& info);
    virtual void onMessage(const mavlink_message_t & message);
public:
    //int stage;
    //bool ready;
    virtual ~SVO() {}
    SVO(ros::NodeHandle & nh, GCSAgent * agent);
};
