#ifndef GCSAGENT_H
#define GCSAGENT_H
#include <map>
#include <ros/ros.h>
#include <mavros_msgs/Mavlink.h>
#include <mavlink/v1.0/ardupilotmega/mavlink.h>

class GCSAgent;

class GCSAgentPlugin
{
protected:
    GCSAgent * _agent;
    void send(const mavlink_message_t & message);
public:
    virtual ~GCSAgentPlugin() {}
    virtual void onMessage(const mavlink_message_t & message) = 0;
    GCSAgentPlugin(GCSAgent * agent, uint8_t msgid);
};

class GCSAgent
{
private:
    friend class GCSAgentPlugin;
    std::map<uint8_t, GCSAgentPlugin *> _plugins;
    ros::Subscriber _subMavlink;
    ros::Publisher _pubMavlink;
    float _infoRate;
    void send(const mavlink_message_t & message);
    void subscribe(uint8_t msgid, GCSAgentPlugin * plugin);
public:
    void publishRosInfo(void);
    void (* _mavlink_send) (const mavlink_message_t & mmsg);
    bool onMavlink(const mavros_msgs::Mavlink::ConstPtr rmsg);
    bool onMavlink(const mavlink_message_t * mmsg);
    void spin(void);
    void testMavlink(void);
    GCSAgent(ros::NodeHandle &nh);
};


#endif // GCSAGENT_H
