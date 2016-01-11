#include "svo.h"
#include <std_msgs/String.h>
#include "gcsagent.h"
#include "mavlink/ros/mavlink.h"

SVO::SVO(ros::NodeHandle & nh, GCSAgent * agent):
    GCSAgentPlugin(agent, MAVLINK_MSG_ID_RPG_SVO_KEY)
{
    subInfo = nh.subscribe("/svo/info", 100, &SVO::onInfo, this);
   	pubKey = nh.advertise<std_msgs::String>("/svo/remote_key", 10);
}

void SVO::onMessage(const mavlink_message_t & message)
{
    char b[2] = {0};
    b[0] = mavlink_msg_rpg_svo_key_get_key(&message);
    std_msgs::String msg;
    msg.data = b;
    pubKey.publish(msg);
}


void SVO::onInfo(const svo_msgs::Info::ConstPtr& info)
{
    mavlink_rpg_svo_info_t msg;
    msg.num_matches      = info->num_matches;
    msg.processing_time  = info->processing_time;
    msg.stage            = info->stage;
    msg.tracking_quality = info->tracking_quality;
    mavlink_message_t packed_msg;
    mavlink_msg_rpg_svo_info_pack(0, 0, &packed_msg,
                                  info->processing_time,
                                  info->num_matches,
                                  info->tracking_quality,
                                  info->stage);
    send(packed_msg);
}

