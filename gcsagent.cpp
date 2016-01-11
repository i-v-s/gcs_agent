#include <unistd.h>
#include <ros/callback_queue.h>
#include "gcsagent.h"
#include <mavros_msgs/mavlink_convert.h>

void GCSAgentPlugin::send(const mavlink_message_t & message)
{
    _agent->send(message);
}

GCSAgentPlugin::GCSAgentPlugin(GCSAgent * agent, uint8_t msgid): _agent(agent)
{
    agent->subscribe(msgid, this);
}


void GCSAgent::send(const mavlink_message_t & message)
{
    mavros_msgs::Mavlink rmsg;
    rmsg.header.stamp = ros::Time::now();
    mavros_msgs::mavlink::convert(message, rmsg);
    _pubMavlink.publish(rmsg);
}

void GCSAgent::subscribe(uint8_t msgid, GCSAgentPlugin * plugin)
{
    _plugins.insert(std::make_pair(msgid, plugin));
}


void GCSAgent::onMavlink(const mavros_msgs::Mavlink::ConstPtr rmsg)
{
    mavlink_message_t mmsg;

    if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
    {
        printf("msgid: %d", mmsg.msgid);
        auto plugin = _plugins.find(mmsg.msgid);
        if(plugin != _plugins.end())
            plugin->second->onMessage(mmsg);
        //gcs_link->send_message(&mmsg, rmsg->sysid, rmsg->compid);
    }
    else
        ROS_ERROR("Packet drop: illegal payload64 size");
}


GCSAgent::GCSAgent(ros::NodeHandle &nh): _infoRate(5)
{
    _subMavlink = nh.subscribe("/mavlink/to", 10, &GCSAgent::onMavlink, this);
    _pubMavlink = nh.advertise<mavros_msgs::Mavlink>("/mavlink/from", 10);
}

void GCSAgent::publishRosInfo()
{
    mavlink_message_t msg;
    mavlink_ros_info_t ros_info;
    gethostname(ros_info.host, sizeof(ros_info.host));
    ros_info.state = 1;
    mavlink_msg_ros_info_encode(0, 0, &msg, &ros_info);
    send(msg);
}

void GCSAgent::spin()
{
    while(ros::ok())
    {
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(1.0 / _infoRate));
        publishRosInfo();
    }
}
