#include <ros/ros.h>
#include <svo_msgs/Info.h>
#include <std_msgs/String.h>
#include "gcsagent.h"
//#include "msf.h"

class FakeSVO
{
    enum Stage {
      STAGE_PAUSED,
      STAGE_FIRST_FRAME,
      STAGE_SECOND_FRAME,
      STAGE_DEFAULT_FRAME,
      STAGE_RELOCALIZING
    };
    Stage _stage;
    ros::Publisher pubInfo;
    ros::Subscriber subKey;
    void onKey(const std_msgs::String::ConstPtr & keyMsg);
public:
    FakeSVO(ros::NodeHandle & nh);
    ~FakeSVO();
};

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
