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

#include "rio_control_node/Robot_Status.h"

ros::NodeHandle* node;

enum RobotState : int
{
    DISABLED = 0,
    TELEOP = 1,
    AUTONOMOUS = 2,
    TEST = 3,
};

static RobotState mPrevRobotState = RobotState::DISABLED;
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
	system("pkill -2 rosbag");
}

void start_ros_bag()
{
	const std::string DEFAULT_BAG_NAME = "/mnt/working/robHatesDashes";

	int file_idx = 0;
	try
	{
		std::string cmd_active_bag = exec("ls /mnt/working/*.bag.active | sort -V | tail -n 1 | cut -c 28- | rev | cut -c 12- | rev");
		std::string cmd_completed_bag = exec("ls /mnt/working/*.bag | sort -V | tail -n 1 | cut -c 28- | rev | cut -c 5- | rev");

		int idx_active = 0;
		int idx_completed = 0;
		try {
			idx_active = std::stoi(cmd_active_bag);
		}
		catch (...)
		{
			idx_active = 0;
		}

		try {
			idx_completed = std::stoi(cmd_completed_bag);
		}
		catch (...)
		{
			idx_completed = 0;
		}
		file_idx = idx_active > idx_completed ? idx_active : idx_completed;
	}
	catch (...)
	{
		file_idx = 0;
	}

	system(("rosbag record -e \"(.*)Diagnostic(.*)\" -O " + (DEFAULT_BAG_NAME + std::to_string(file_idx + 1) + ".bag") + "  &").c_str());
}

void sync_fs()
{
	system("sudo sync");
}

static ros::Time time_in_disabled(0);
void robot_status_callback (const rio_control_node::Robot_Status &msg)
{
	(void) msg;
	mRobotState = (RobotState)msg.robot_state;
	if (mRobotState == RobotState::DISABLED && mPrevRobotState != RobotState::DISABLED)
	{
		time_in_disabled = ros::Time::now();
	}
	else if (mRobotState != RobotState::DISABLED)
	{
		time_in_disabled = ros::Time(0);
	}

	if (time_in_disabled != ros::Time(0) && (ros::Time::now() - time_in_disabled) > ros::Duration(10))
	{
		stop_ros_bag();
		sync_fs();
		start_ros_bag();
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

	start_ros_bag();	//Initial start

	ros::spin();
	stop_ros_bag();
	sync_fs();
	return 0;
}