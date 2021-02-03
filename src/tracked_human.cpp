#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "hanp_msgs/TrackedHumans.h"
#include "hanp_msgs/TrackedHuman.h"
#include "hanp_msgs/TrackedSegment.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/TwistStamped.h"

ros::Publisher pub_tracked_human;
geometry_msgs::Twist vel;
geometry_msgs::Pose pose;

hanp_msgs::TrackedHumans tracked_humans_msg;
hanp_msgs::TrackedHuman tracked_human_msg;
hanp_msgs::TrackedSegment tracked_segment_msg;

void humanPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	pose = msg->pose;
}

void humanVelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	vel = msg->twist;
}

void publishTrackedHuman()
{	
	tracked_segment_msg.pose.pose = pose;
	tracked_segment_msg.twist.twist = vel;

	tracked_human_msg.segments.clear();
	tracked_human_msg.segments.push_back(tracked_segment_msg);

	tracked_humans_msg.header.stamp = ros::Time::now();
	tracked_humans_msg.humans.clear();
	tracked_humans_msg.humans.push_back(tracked_human_msg);

	pub_tracked_human.publish(tracked_humans_msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tracked_human");

	ros::NodeHandle nh;

	ros::Subscriber sub_human_pose = nh.subscribe("/morse/human_pose", 100, humanPoseCallback);
	ros::Subscriber sub_human_vel = nh.subscribe("/morse/human_vel", 100, humanVelCallback);
	pub_tracked_human = nh.advertise<hanp_msgs::TrackedHumans>("/tracked_humans", 100);

	tracked_humans_msg.header.frame_id = "map";
	tracked_human_msg.track_id = 1;
	tracked_segment_msg.type = 1;

	ros::Rate rate(60);
	while(ros::ok())
	{
		publishTrackedHuman();

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
