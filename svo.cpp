#include "svo.h"
#include "gcsagent.h"
#include <mavlink/v1.0/ardupilotmega/mavlink.h>

void FakeSVO::onKey(const std_msgs::String::ConstPtr & keyMsg)
{
    static bool b = false;
    std::string key = keyMsg->data;
    switch(*key.c_str())
    {
    case 'R':
    case 'r':
        ROS_INFO("FakeSVO: R");
        _stage = STAGE_PAUSED;
        break;
    case 'S':
    case 's':
        ROS_INFO("FakeSVO: S");
        _stage = b ? STAGE_RELOCALIZING : STAGE_DEFAULT_FRAME;
        b = !b;
        break;
    }
    svo_msgs::Info info;
    info.stage = _stage;
    pubInfo.publish(info);
}

FakeSVO::FakeSVO(ros::NodeHandle & nh)
{
    pubInfo = nh.advertise<svo_msgs::Info>("/svo/info", 10);
    subKey = nh.subscribe("/svo/remote_key", 10, &FakeSVO::onKey, this);
}

FakeSVO::~FakeSVO()
{

}

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
    //ROS_INFO("SVO Info received, stage %d", info->stage);
    mavlink_rpg_svo_info_t msg;
    msg.num_matches      = info->num_matches;
    msg.processing_time  = info->processing_time;
    msg.stage            = info->stage;
    msg.tracking_quality = info->tracking_quality;
    mavlink_message_t packed_msg;
    mavlink_msg_rpg_svo_info_pack_chan(1, 1, 0, &packed_msg,
                                  info->processing_time,
                                  info->num_matches,
                                  info->tracking_quality,
                                  info->stage);
    send(packed_msg);
}

