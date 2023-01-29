#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <atomic>
#include <sys/stat.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>

#include "ck_ros_base_msgs_node/Robot_Status.h"
#include "boost/date_time/posix_time/posix_time.hpp"

ros::NodeHandle* node;

enum RobotState : int
{
    DISABLED = 0,
    TELEOP = 1,
    AUTONOMOUS = 2,
    TEST = 3,
};

static RobotState mRobotState = RobotState::DISABLED;

std::string exec(const char* cmd)
{
	std::array<char, 128> buffer;
	std::string result;
	std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
	if (!pipe)
	{
		throw std::runtime_error("popen() failed!");
	}
	while (fgets(buffer.data(), buffer.size(), pipe.get()))
	{
		result += buffer.data();
	}
	return result;
}

void stop_ros_bag()
{
    ROS_INFO("Stopping the recording");
	system("pkill -2 rosbag");
}

void start_ros_bag()
{
	const std::string DEFAULT_BAG_NAME = "/mnt/working/ros_log_";

    boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
    std::string date_string = boost::posix_time::to_iso_string(my_posix_time);

    std::stringstream s;
    s << "rosbag record --tcpnodelay -a -O " << DEFAULT_BAG_NAME << date_string << ".bag &";

	system(s.str().c_str());

    ROS_INFO("Starting a recording at: %s", s.str().c_str());
}

void sync_fs()
{
	// system("sudo sync");
}

void robot_status_callback (const ck_ros_base_msgs_node::Robot_Status &msg)
{
	static ros::Time time_in_disabled = ros::Time::now();
	mRobotState = (RobotState)msg.robot_state;
	static RobotState mPrevRobotState = RobotState::DISABLED;
	static bool started = false;

	if (mRobotState != RobotState::DISABLED && mPrevRobotState == RobotState::DISABLED && !started)
	{
		started = true;
		start_ros_bag();
	}

	if (mRobotState != RobotState::DISABLED)
	{
		time_in_disabled = ros::Time::now();
	}

	if (time_in_disabled != ros::Time(0) && (ros::Time::now() - time_in_disabled) > ros::Duration(10))
	{
		stop_ros_bag();
		started = false;
		sync_fs();
		time_in_disabled = ros::Time(0);
	}
	mPrevRobotState = mRobotState;
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "logger_node");

	ros::NodeHandle n;

	node = &n;

	static ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 1, robot_status_callback);
	stop_ros_bag();

	ros::spin();
	stop_ros_bag();
	sync_fs();
	return 0;
}