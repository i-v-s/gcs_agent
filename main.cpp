#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "gcsagent.h"
#include "svo.h"
#include "launcher.h"


/**
 * @brief MAVROS GCS proxy
 * @file gcs_bridge.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <ros/ros.h>
#include <ros/console.h>

//#include <mavros/utils.h>
//#include <mavros/mavlink_diag.h>
#include <mavros_msgs/mavlink_convert.h>
#include <mavconn/interface.h>

class MavlinkDiag : public diagnostic_updater::DiagnosticTask
{
public:
	explicit MavlinkDiag(std::string name);

	void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

	void set_mavconn(const mavconn::MAVConnInterface::Ptr &link) {
		weak_link = link;
	}

	void set_connection_status(bool connected) {
		is_connected = connected;
	}

private:
	mavconn::MAVConnInterface::WeakPtr weak_link;
	unsigned int last_drop_count;
	std::atomic<bool> is_connected;
};

MavlinkDiag::MavlinkDiag(std::string name) :
	diagnostic_updater::DiagnosticTask(name),
	last_drop_count(0),
	is_connected(false)
{ };

void MavlinkDiag::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	if (auto link = weak_link.lock()) {
		auto mav_status = link->get_status();
		auto iostat = link->get_iostat();

		stat.addf("Received packets:", "%u", mav_status.packet_rx_success_count);
		stat.addf("Dropped packets:", "%u", mav_status.packet_rx_drop_count);
		stat.addf("Buffer overruns:", "%u", mav_status.buffer_overrun);
		stat.addf("Parse errors:", "%u", mav_status.parse_error);
		stat.addf("Rx sequence number:", "%u", mav_status.current_rx_seq);
		stat.addf("Tx sequence number:", "%u", mav_status.current_tx_seq);

		stat.addf("Rx total bytes:", "%u", iostat.rx_total_bytes);
		stat.addf("Tx total bytes:", "%u", iostat.tx_total_bytes);
		stat.addf("Rx speed:", "%f", iostat.rx_speed);
		stat.addf("Tx speed:", "%f", iostat.tx_speed);

		if (mav_status.packet_rx_drop_count > last_drop_count)
			stat.summaryf(1, "%d packeges dropped since last report",
				mav_status.packet_rx_drop_count - last_drop_count);
		else if (is_connected)
			stat.summary(0, "connected");
		else
			// link operational, but not connected
			stat.summary(1, "not connected");

		last_drop_count = mav_status.packet_rx_drop_count;
	} else {
		stat.summary(2, "not connected");
	}
}

//using namespace mavros;
using namespace mavconn;

ros::Publisher mavlink_pub;
ros::Subscriber mavlink_sub;
MAVConnInterface::Ptr gcs_link;

GCSAgent * agent = nullptr;
SVO * svo = nullptr;
Launcher * launcher = nullptr;

void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid) {
    if(agent->onMavlink(mmsg)) return;
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg);
	mavlink_pub.publish(rmsg);
}

void mavlink_send(const mavlink_message_t & mmsg)
{
    gcs_link->send_message(&mmsg, mmsg.sysid, mmsg.compid);
}

void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg) {
	mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		gcs_link->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	else
		ROS_ERROR("Packet drop: illegal payload64 size");
}

int gcs_bridge(ros::NodeHandle & nh)
{
	ros::NodeHandle mavlink_nh("mavlink");
    diagnostic_updater::Updater updater;
    MavlinkDiag gcs_link_diag("GCS agent");

	std::string gcs_url;
	nh.param<std::string>("gcs_url", gcs_url, "udp://@");

	try {
		gcs_link = MAVConnInterface::open_url(gcs_url);
        gcs_link_diag.set_mavconn(gcs_link);
        gcs_link_diag.set_connection_status(true);
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("GCS_AGENT: %s", ex.what());
		return 1;
	}

	mavlink_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("to", 10);
	gcs_link->message_received.connect(mavlink_pub_cb);

	// prefer UDPROS, but allow TCPROS too
	mavlink_sub = mavlink_nh.subscribe("from", 10, mavlink_sub_cb,
		ros::TransportHints()
			.unreliable().maxDatagramSize(1024)
			.reliable());

	// setup updater
    updater.setHardwareID(gcs_url);
    updater.add(gcs_link_diag);

	// updater spinner
    auto diag_timer = nh.createTimer(ros::Duration(0.5),
			[&](const ros::TimerEvent &evt) {
				updater.update();
				agent->publishRosInfo();
			});
    diag_timer.start();

	ros::spin();
	return 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gcs_agent");
    ros::NodeHandle nh("~");
    agent = new GCSAgent(nh);
    agent->_mavlink_send = &mavlink_send;
    svo = new SVO(nh, agent);
    launcher = new Launcher(nh, agent);
    int r = gcs_bridge(nh);
    delete svo;
    delete launcher;
    delete agent;
    return r;
}

