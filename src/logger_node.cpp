#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>
#include <atomic>

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

void stop_ros_bag()
{
	system("pkill -2 rosbag");
}

void start_ros_bag()
{
	system("rosbag record -e \"(.*)Diagnostic(.*)\" -o /mnt/working/ &");

	// system("rosbag record -a -o /mnt/working/ &");
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