#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <typeinfo>
#include "geometry_msgs/PoseStamped.h"
#include "boost/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include "mavros_msgs/State.h"

using namespace std;

class ROS_SUB {

	public:
		ROS_SUB();
		void topic_cb_modelPose(const geometry_msgs::PoseStampedConstPtr& data);
		void topic_cb_qrPose(const geometry_msgs::PoseStampedConstPtr& data);
		void topic_cb_qrData(const std_msgs::String data);
		void mavros_state_cb( mavros_msgs::State mstate);
		void run();
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub_modelPose;
		ros::Subscriber _topic_sub_qrPose;
		ros::Subscriber _topic_sub_qrData;
    ros::Subscriber _mavros_state_sub;
		tf::TransformListener listener;
		tf::Transform *transform_baselink_map;
		tf::StampedTransform transform_qr_world;
		int qr_code_reading = 0;

		ros::Publisher publisher_qr_position;
		
		mavros_msgs::State _mstate;
};


ROS_SUB::ROS_SUB() {
	_topic_sub_modelPose = _nh.subscribe("/mavros/local_position/pose", 0, &ROS_SUB::topic_cb_modelPose, this);
	_mavros_state_sub = _nh.subscribe("/mavros/state", 0, &ROS_SUB::mavros_state_cb, this);

	transform_baselink_map = new tf::Transform();
}

void ROS_SUB::topic_cb_modelPose(const geometry_msgs::PoseStampedConstPtr& data) {
	if(_mstate.mode != "AUTO.LAND"){ //alternatively _mstate.mode == "OFFBOARD" 
	
		static tf::TransformBroadcaster br;

		tf::Vector3 position(data->pose.position.x, data->pose.position.y, data->pose.position.z);
		tf::Quaternion quaternion(data->pose.orientation.x, data->pose.orientation.y, data->pose.orientation.z, data->pose.orientation.w);
		quaternion.normalize();
		transform_baselink_map->setOrigin(position);
		transform_baselink_map->setRotation(quaternion);

		br.sendTransform(tf::StampedTransform(*transform_baselink_map, ros::Time::now(), "map", "base_link"));
	}
}

void ROS_SUB::mavros_state_cb( mavros_msgs::State mstate)	{
	_mstate = mstate;
}


void ROS_SUB::run(){
	ros::spin();
}


int main( int argc, char** argv ) {
	ros::init(argc, argv, "tf_map2base_node");
	ROS_SUB rs;
	rs.run();
	return 0;
}
