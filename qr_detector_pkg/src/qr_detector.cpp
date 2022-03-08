/*********************************************************************
The aim of this node is to use the information provided by VISP (camera frame)
and transform them in order to behave like a server containing all the
position (map frame) of the QR codes that have been readed.
*********************************************************************/

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <iostream>
#include "gazebo_msgs/ModelStates.h"
#include <typeinfo>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <qr_detector_pkg/qr_detection_msg.h>
#include "boost/thread.hpp"
#include <tf/transform_listener.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <qr_detector_pkg/activate_service.h>
#include <qr_detector_pkg/qr_detection_msg.h>
#include <qr_detector_pkg/qr_position_service.h>
#include <deque>  

#define NUM_QRCODE 7
#define ACTIVATE_QR_SERVICE "/qr_detector/activate"
#define QRCODE_DETECTED_MASK_TOPIC "/qr_detector/detected_QRcode_mask"
#define QRCODE_POSITION_SERVICE "/qr_detector/qr_code_pos_map"
#define QRCODE_POSITION_TOPIC "/qr_detector/qr_pos_mov_average"
#define VISP_OBJ_POS_TOPIC "/visp_auto_tracker/object_position"
#define VISP_CODE_MSG_TOPIC "/visp_auto_tracker/code_message"
#define MEASURE_BOUND 20.0

using namespace std;


/*************************************************************************************************************************************************
Class containing the the function prototypes and the instance attributes 
_topic_sub_qrPose: subscriber to obtain the QR position from the VISP topic (callback: topic_cb_qrPose)
_topic_sub_qrData: subscriber to obtain the QR code from the VISP topic (callback: topic_cb_qrData)
_publisher_detected_QRcode: publish a mask representing which QR code has been detected
qrposition_service: Service to give the position (in map frame) of the desired qr code (callback: qrposition_service_cb)
active_service: Service to activate the entire node (callback: activate_service_cb)
publisher_qr_position: publisher the current QR position detected 

qr_code_reading: code of the QR code that is currently detected
filtered_qr_positions: array containing the filtered QR position 
detected_QRcode_mask: mask containing the QR code which has been readed (example 7 QR code, a possibile mask could be 0100100 showing that only 2 qr code are readed)
qr_pos_deque: deque that contains a maximum of 5 QR code last detected position by VISP, needed to compute the moving average of the position  
current_qr_average: stores the QR code for which the moving average of the positions is computed and streamed by topic
*************************************************************************************************************************************************/
class ROS_SUB {

	public:
		ROS_SUB();
		void topic_cb_qrPose(const geometry_msgs::PoseStampedConstPtr& data);
		void topic_cb_qrData(const std_msgs::String data);
		bool activate_service_cb(qr_detector_pkg::activate_service::Request &req, qr_detector_pkg::activate_service::Response &res);
		bool qrposition_service_cb(qr_detector_pkg::qr_position_service::Request &req, qr_detector_pkg::qr_position_service::Response &res);
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub_qrPose;
		ros::Subscriber _topic_sub_qrData;
		ros::Publisher _publisher_detected_QRcode;
		ros::ServiceServer qrposition_service;
		ros::ServiceServer active_service;
		ros::Publisher publisher_qr_position;

		tf::TransformListener listener;
		int qr_code_reading = 0;
		geometry_msgs::Point filtered_qr_positions[NUM_QRCODE];
		std_msgs::Int32 detected_QRcode_mask;		
		std::deque<geometry_msgs::Point> qr_pos_deque;
		int current_qr_average = 0;
};


/*************************************************************************************************************************************************
Class Constructor. 
Two server are advertised 
	1. service to "activate" the entire node
	2. service to give the desired QR position
The filtered qr code position are inizialized. 
*************************************************************************************************************************************************/
ROS_SUB::ROS_SUB() {
	active_service = _nh.advertiseService(ACTIVATE_QR_SERVICE, &ROS_SUB::activate_service_cb, this);
	qrposition_service = _nh.advertiseService(QRCODE_POSITION_SERVICE, &ROS_SUB::qrposition_service_cb, this);


	detected_QRcode_mask.data = 0;

	for (int i=0; i<NUM_QRCODE; i++) {
		filtered_qr_positions[i].x = 0;
		filtered_qr_positions[i].y = 0;
		filtered_qr_positions[i].z = 0;
	}
}



/*************************************************************************************************************************************************
Service callback. Used to "activate" the node and "deactivate" the node.
There are 2 subscriber
	1. subscription to the VISP detected QR pose
	2. subscription to the VISP detected QR code
There are 2 publisher
	1. publishes a mask containing the readed QR coded 
	2. publishes the instant measure of the current object detection (the measures is filtered using a moving average)
*************************************************************************************************************************************************/

bool ROS_SUB::activate_service_cb(qr_detector_pkg::activate_service::Request &req, qr_detector_pkg::activate_service::Response &res) {
	if (req.activate) {
		_topic_sub_qrPose = _nh.subscribe(VISP_OBJ_POS_TOPIC, 0, &ROS_SUB::topic_cb_qrPose, this);
		_topic_sub_qrData = _nh.subscribe(VISP_CODE_MSG_TOPIC, 0, &ROS_SUB::topic_cb_qrData, this);
		_publisher_detected_QRcode = _nh.advertise<std_msgs::Int32>(QRCODE_DETECTED_MASK_TOPIC,1);
		publisher_qr_position = _nh.advertise<qr_detector_pkg::qr_detection_msg>(QRCODE_POSITION_TOPIC,1);

		cout<<"--- qr detector service activated---"<<endl;
		res.activated = true;
	}
	else {
		_topic_sub_qrPose.shutdown();
		_topic_sub_qrData.shutdown();
		_publisher_detected_QRcode.shutdown();
		publisher_qr_position.shutdown();
		cout<<"--- qr detector service shutdown---"<<endl;
		res.activated = false;
	}
	return true;
}


/*************************************************************************************************************************************************
Topic callback from the VISP topic containing the QR code. It Updates the QR code of the current reading. 
*************************************************************************************************************************************************/
void ROS_SUB::topic_cb_qrData(const std_msgs::String data) {
	qr_code_reading = atoi(data.data.c_str());
}


/*************************************************************************************************************************************************
Topic callback from the VISP topic containing the position in camera frame of the current QR code detected.

It uses the TF package to obtain the transformation from the camera from to the map frame. 
This function does three main steps
	- stores the avaraged position of each QR code readed.
	- publishes the updated mask containing which QR code has been readed
	- publishes the current QR code position detected a little filtered (a moving avarage is used).
*************************************************************************************************************************************************/
void ROS_SUB::topic_cb_qrPose(const geometry_msgs::PoseStampedConstPtr& data) {
	tf::StampedTransform transformation;
	static int counters[NUM_QRCODE] = {0};			//counter to compute avarage

	if (qr_code_reading!=0) {
		int index = qr_code_reading-1;
		counters[index]++;

		tf::Vector3 position(data->pose.position.x, data->pose.position.y, data->pose.position.z);

		ros::Time now = ros::Time::now();
		if (listener.waitForTransform("map", "rgb_camera_frame", now, ros::Duration(0.1))) {
			listener.lookupTransform("map", "rgb_camera_frame", now, transformation);
			tf::Vector3 position_transf;
			position_transf = transformation*(position);

			cout<<"Tracking qr code "<<qr_code_reading<<": "<<position.getX()<<" - "<<position.getY()<<" - "<<position.getZ()<<endl;
			geometry_msgs::Point reading_qr_position;
			reading_qr_position.x = position_transf.getX();
			reading_qr_position.y = position_transf.getY();
			reading_qr_position.z = position_transf.getZ();
			
			cout<<"Tracking qr code "<<qr_code_reading<<": "<<position_transf.getX()<<" - "<<position_transf.getY()<<" - "<<position_transf.getZ()<<endl;
			if (fabs(reading_qr_position.x) < MEASURE_BOUND && fabs(reading_qr_position.y) < MEASURE_BOUND  && fabs(reading_qr_position.z) < MEASURE_BOUND ) {

				filtered_qr_positions[index].x = filtered_qr_positions[index].x + 1.0/counters[index] * (reading_qr_position.x - filtered_qr_positions[index].x);
				filtered_qr_positions[index].y = filtered_qr_positions[index].y + 1.0/counters[index] * (reading_qr_position.y - filtered_qr_positions[index].y);
				filtered_qr_positions[index].z = filtered_qr_positions[index].z + 1.0/counters[index] * (reading_qr_position.z - filtered_qr_positions[index].z);
				cout<<"-- avarage "<<counters[index]<<": "<<filtered_qr_positions[index].x<<" - "<<filtered_qr_positions[index].y<<" - "<<filtered_qr_positions[index].z<<endl;
				cout<<endl;
				detected_QRcode_mask.data = detected_QRcode_mask.data | (int) pow(2,index);

				_publisher_detected_QRcode.publish(detected_QRcode_mask);				
				
				/*computing and sending qr code moving average position*/
				qr_detector_pkg::qr_detection_msg msg;
				msg.qr_code = qr_code_reading;
				if(current_qr_average != qr_code_reading){ //reset deque 
					cout<<"clearing queue! current QR value: "<<current_qr_average<<"new QR value: "<<qr_code_reading<<endl;
					current_qr_average = qr_code_reading;
					qr_pos_deque.clear();
				}
				qr_pos_deque.push_back(reading_qr_position); 
				
				if(qr_pos_deque.size() == 6) //average on last 5 position 
					qr_pos_deque.pop_front();
								
				deque<geometry_msgs::Point>::iterator it; 
    		cout<<"moving average:"<<endl;
    		for (it = qr_pos_deque.begin(); it != qr_pos_deque.end(); ++it){
					msg.qr_position.x += (*it).x;
					msg.qr_position.y += (*it).y;
					cout<<"x: "<<(*it).x<<"\t"<<"y: "<<(*it).y<<endl; //test 
    		} 
    		cout<<"end"<<endl;
				msg.qr_position.x = msg.qr_position.x/qr_pos_deque.size();
				msg.qr_position.y = msg.qr_position.y/qr_pos_deque.size();
				cout<<"computed: "<<msg.qr_position.x<<"\t"<<msg.qr_position.y<<endl; //test
				publisher_qr_position.publish(msg);
			}
			else { cout<<"Measure has been discarded"<<endl<<endl; }
		}
		else {
			cout<<"Transformation from camera to map frame cannot be performed"<<endl;
		}
	}
}


/*************************************************************************************************************************************************
Service callback. The client send a QR code and the service return its (avaraged) position in the map frame. 
*************************************************************************************************************************************************/
bool ROS_SUB::qrposition_service_cb(qr_detector_pkg::qr_position_service::Request &req, qr_detector_pkg::qr_position_service::Response &res) {
	res.qr_position = filtered_qr_positions[req.qr_code-1];
	return true;
}

int main( int argc, char** argv ) {
	ros::init(argc, argv, "qr_detector_node");
	ROS_SUB rs;
	ros::spin();
	return 0;
}
