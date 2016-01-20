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

void GCSAgent::testMavlink(void)
{
    // Create ros_info
    mavlink_message_t mmsg;
    char host[32];
    int hostId = 0;
    gethostname(host, sizeof(host));
    mavlink_msg_ros_info_pack_chan(1, 1, 0, &mmsg,
                                   hostId,
                                   1,
                                   host,
                                   1);

    send(mmsg);
    //>>>> mavlink_message_t mmsg

    uint8_t data[MAVLINK_MAX_PACKET_LEN + 2 + 7];
    int len = mavlink_msg_to_send_buffer(data, &mmsg);

    //>>>> uint8_t data[]

    mavlink_message_t r;
    mavlink_status_t status;
    for(int x = 0; x < len; x++)
    {
        if(int ok = mavlink_parse_char(0, data[x], &r, &status))
            ROS_INFO("Test passed");
    }
}


void GCSAgent::send(const mavlink_message_t & message)
{
    if(_mavlink_send)
    {
        _mavlink_send(message);
        return;
    }

    auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

    //mavros_msgs::Mavlink rmsg;
    rmsg->header.stamp = ros::Time::now();
    mavros_msgs::mavlink::convert(message, *rmsg);

    _pubMavlink.publish(rmsg);
}

void GCSAgent::subscribe(uint8_t msgid, GCSAgentPlugin * plugin)
{
    _plugins.insert(std::make_pair(msgid, plugin));
}

bool GCSAgent::onMavlink(const mavlink_message_t * mmsg)
{
    auto plugin = _plugins.find(mmsg->msgid);
    if(plugin == _plugins.end()) return false;
    plugin->second->onMessage(*mmsg);
    return true;
}

bool GCSAgent::onMavlink(const mavros_msgs::Mavlink::ConstPtr rmsg)
{
    mavlink_message_t mmsg;
    auto plugin = _plugins.find(rmsg->msgid);
    if(plugin == _plugins.end()) return false;
    if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
    {
        //ROS_INFO("msgid: %d", mmsg.msgid);
        plugin->second->onMessage(mmsg);
    }
    else
        ROS_ERROR("Packet drop: illegal payload64 size");
    return true;
}


GCSAgent::GCSAgent(ros::NodeHandle &nh): _infoRate(5), _mavlink_send(nullptr)
{
    //_subMavlink = nh.subscribe("/mavlink/to", 10, &GCSAgent::onMavlink, this);
    //_pubMavlink = nh.advertise<mavros_msgs::Mavlink>("/mavlink/from", 10);

    //testMavlink();
}

void GCSAgent::publishRosInfo()
{
    mavlink_message_t msg;
    char host[sizeof(mavlink_ros_info_t::host)];
    gethostname(host, sizeof(host));
    int hostId = 0;
    mavlink_msg_ros_info_pack_chan(1, 1, 0, &msg, hostId, 1, host, 1);

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
