#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include <iostream>
#include <Eigen/Dense>
#include "boost/thread.hpp"
#include <boost/algorithm/string.hpp>
#include <px4_planner/planner_commander_service.h>
#include <qr_detector_pkg/activate_service.h>
#include <qr_detector_pkg/qr_position_service.h>

#define CLIENTSERVER_NAME "planner_commander_service"
#define STARTING_POSITION_X 0.0
#define STARTING_POSITION_Y 0.0
#define NUM_QRCODE 7
#define QRCODE_POSITION_SERVICE "/qr_detector/qr_code_pos_map"
#define ACTIVATE_QR_SERVICE "/qr_detector/activate"
#define QRCODE_DETECTED_MASK_TOPIC "/qr_detector/detected_QRcode_mask"
#define TIME_TO_WAIT_ON_QRCODE 1.0



#include "geometry_msgs/Point.h"


using namespace std;


class MASTER {
	public:
		MASTER();
		void run();
		void key_input();
		void master_menu();

		bool exploration_phase();
		bool exploration_phase2();
		bool landOnQRcode(float x, float y, int qr_code);
		bool landOnPoint(float x, float y);
		bool reachPoint(float x, float y);
		void compute_waypoints(const float& , const float& , const float& , const float&);
		void print_waypoints();
		void topic_cb_detectedQRcode(std_msgs::Int32 data);
		std::string get_this_input();

	private:
		ros::NodeHandle _nh;
		int _tot_number_qr_code;
		ros::ServiceClient _client_qrdetector;
		ros::ServiceClient _client_qrPosition;
		ros::ServiceClient _client_planner;
		ros::ServiceClient _client_planner_abort;
		ros::Subscriber _subscriber_detected_QRcode;

		bool _exploration_completed = false;
		bool _abort_request = false;
		Eigen::VectorXd _x_waypoints;
		Eigen::VectorXd _y_waypoints;
		int qr_detected_mask = 0;
		int qr_code_finded = 0;
		std::string inputString = "";
		bool _new_key_input = 0;
		bool _need_input = false;
};


MASTER::MASTER() {

//	PLANNER SERVICES
	_client_planner = _nh.serviceClient<px4_planner::planner_commander_service>(CLIENTSERVER_NAME);
	_client_planner_abort = _nh.serviceClient<px4_planner::planner_commander_service>("planner_abort");


	_client_qrdetector = _nh.serviceClient<qr_detector_pkg::activate_service>(ACTIVATE_QR_SERVICE);
	_client_qrPosition = _nh.serviceClient<qr_detector_pkg::qr_position_service>(QRCODE_POSITION_SERVICE);

	_subscriber_detected_QRcode = _nh.subscribe(QRCODE_DETECTED_MASK_TOPIC,1, &MASTER::topic_cb_detectedQRcode, this);

	//_nh.getParam("/master_exe/_tot_number_qr_code", _tot_number_qr_code);
}

void MASTER::topic_cb_detectedQRcode(std_msgs::Int32 data) {
	int new_qr_detected_mask = data.data;

	if(qr_detected_mask != new_qr_detected_mask){
		int new_qr_code_mask = qr_detected_mask ^ new_qr_detected_mask; //XOR
		for (int qr_code=1; qr_code <= NUM_QRCODE; qr_code++) {
			if ((new_qr_code_mask & int(pow(2,qr_code-1))) != 0) {
				cout<<"---------- New QR code detected: "<<qr_code<<endl;
				qr_code_finded++;
			}
		}
		qr_detected_mask = new_qr_detected_mask;
	}

}

void MASTER::key_input(){
	ros::Rate r(2);
	while (ros::ok()) {
		if (_need_input) {
			std::getline(cin, inputString);
			_new_key_input = true;
			_need_input = false;
		}
		r.sleep();
	}
}

std::string MASTER::get_this_input(){
	ros::Rate r(2);
	_need_input = true;
	while (ros::ok() && !_new_key_input) {
		r.sleep();
	}
	_new_key_input = false;
	return inputString;

}

void MASTER::master_menu(){
	ros::Rate r(10);
	while (ros::ok() && (inputString != "k")) {
		//qr_detector_pkg::activate_service srv; srv.request.activate = true; _client_qrdetector.call(srv); _client_qrfilter.call(srv);
		cout<<endl<<"Here, a list of the possible actions: "<<endl;
		cout<<"\t 1. Start exploring action"<<endl;
		cout<<"\t 2. Start exploring action (no QR code detection)"<<endl;
		cout<<"\t 3. Show the QR code obtained positions"<<endl;
		cout<<"\t 4. Go on point. Its position is given by the user."<<endl;
		cout<<"\t 5. Land on point. Its position is given by the user."<<endl;
		cout<<"\t 6. Reach a QR code. Its position is given by the user."<<endl;
		cout<<"\t 7. Reach a QR code sequence. The list is given by the user."<<endl;
		cout<<"\t 8. Reach the QR code TEST sequence"<<endl;
		cout<<"\t 9. Activate QR service"<<endl;
		cout<<"\t -- Type 'k' to exit --"<<endl;
		cout<<"-----------------> Make your choice: ";


		std::string menuChoice = get_this_input();

		if (menuChoice=="1") { //Start exploring action
			qr_detector_pkg::activate_service srv;
			srv.request.activate = true;
			if (_client_qrdetector.call(srv)) {
				cout<<"QR detector correctly activated."<<endl;
				if(exploration_phase()) { cout<<"--> Exploration correctly completed. "<<endl<<endl;}
				else { cout<<"--> Exploration has failed. "<<endl<<endl; }
				srv.request.activate = false;
				_client_qrdetector.call(srv);
			}
			else { 	cout<<"Failed to activate the QR detector."<<endl; 	}
		}
		else if (menuChoice=="3") { //Show the QR code obtained positions
			qr_detector_pkg::activate_service srv_activate;
			srv_activate.request.activate = true;
			if (_client_qrdetector.call(srv_activate)) {
				qr_detector_pkg::qr_position_service srv;
				geometry_msgs::Point point;
				for (int qr_code=1; qr_code <= NUM_QRCODE; qr_code++) {
					if ((qr_detected_mask & int(pow(2,qr_code-1))) != 0) {
						srv.request.qr_code = qr_code;
						if (_client_qrPosition.call(srv)) {
							point = srv.response.qr_position;
							cout<<"--- QR code : "<<qr_code;
							cout<<"-------> x: "<<point.x<<" ,  y: "<<point.y<<" ,  z:  "<<point.z<<endl;
						}
						else {	cout<<"Failed to call qr_position_service"<<endl;		}
					}
					else cout<<"QR code "<<qr_code<<" has not been detected yet."<<endl;
				}
			}
			else { 	cout<<"Failed to activate the QR detector."<<endl; 	}
		}
		else if (menuChoice=="7") {
			qr_detector_pkg::activate_service srv_activate;
			srv_activate.request.activate = true;
			if (_client_qrdetector.call(srv_activate)) {
				cout<<"QR detector correctly activated."<<endl;
				qr_detector_pkg::qr_position_service srv;
				geometry_msgs::Point point;
				cout<<"Write down here the QR code sequence you want to nagivate"<<endl; 	cout<<"\t\t Example: 2 1 3 4"<<endl;
				cout<<"Qr code sequence: ";
				std::string qr_code_sequence_str = get_this_input();
				std::vector<std::string> qr_code_sequence;
				boost::split(qr_code_sequence, qr_code_sequence_str, [](char c) {return c==' ';});

				for (int i=0; i<qr_code_sequence.size(); i++){
					int qr_code = atoi(qr_code_sequence[i].c_str());
					if ((qr_detected_mask & int(pow(2,qr_code-1))) != 0) {
						srv.request.qr_code = qr_code;
						if (_client_qrPosition.call(srv)) {
							point = srv.response.qr_position;
							cout<<"--- QR code : "<<qr_code;
							cout<<"------------> x: "<<point.x<<" ,  y: "<<point.y<<" ,  z:  "<<point.z<<endl;
						}
						else {
							cout<<"Failed to call qr_position_service"<<endl;
						}
						landOnQRcode(point.x, point.y, qr_code);
						ros::Duration(TIME_TO_WAIT_ON_QRCODE).sleep();
					}
					else cout<<"QR code "<<qr_code<<" has not been detected yet, it will be skipped"<<endl;
				}
				landOnPoint(STARTING_POSITION_X, STARTING_POSITION_Y);
				srv_activate.request.activate = false;
				_client_qrdetector.call(srv_activate);
			}
			else { 	cout<<"Failed to activate the QR detector."<<endl; 	}
		}
		else if (menuChoice=="2") { //Start exploring action
			if(exploration_phase()) { cout<<"--> Exploration correctly completed. "<<endl<<endl;}
			else { cout<<"--> Exploration has failed. "<<endl<<endl; }
		}
		else if (menuChoice=="9") {
			qr_detector_pkg::activate_service srv;
			srv.request.activate = true;
			if (_client_qrdetector.call(srv)) {
				cout<<"QR detector correctly activated.";
			}
			else { 	cout<<"Failed to activate the QR detector."<<endl; 	}
		}
		else if (menuChoice=="6") {
			qr_detector_pkg::activate_service srv;
			srv.request.activate = true;
			if (_client_qrdetector.call(srv)) {
				cout<<"QR detector correctly activated."<<endl;
				cout<<"Write here the the QR code: ";
				int qr_code = stof(get_this_input());
				cout<<"\t x coordinate (map frame) (example 6.12): ";
				float x_cord = stof(get_this_input());
				cout<<"\t y coordinate (map frame): ";
				float y_cord = stof(get_this_input());
				landOnQRcode(x_cord, y_cord, qr_code);
				srv.request.activate = false;
				_client_qrdetector.call(srv);
			}
			else { 	cout<<"Failed to activate the QR detector."<<endl; 	}
		}
		else if (menuChoice=="5") {
			cout<<"\t x coordinate (map frame) (example 6.12): ";
			float x_cord = stof(get_this_input());
			cout<<"\t y coordinate (map frame): ";
			float y_cord = stof(get_this_input());
			landOnPoint(x_cord, y_cord);
		}
		else if (menuChoice=="4") {
			cout<<"\t x coordinate (map frame) (example 6.12): ";
			float x_cord = stof(get_this_input());
			cout<<"\t y coordinate (map frame): ";
			float y_cord = stof(get_this_input());
			reachPoint(x_cord, y_cord);
		}
		else if (menuChoice=="8") {
			qr_detector_pkg::activate_service srv_activate;
      srv_activate.request.activate = true;
      if (_client_qrdetector.call(srv_activate)) {
        landOnQRcode(8.71-0.5, 5.47-1.5 , 2);
        ros::Duration(TIME_TO_WAIT_ON_QRCODE).sleep();
        landOnQRcode(9.76-0.5, 1.51-1.5, 3);
        ros::Duration(TIME_TO_WAIT_ON_QRCODE).sleep();
        landOnQRcode(4.91-0.5, 9.45-1.5, 4);
        ros::Duration(TIME_TO_WAIT_ON_QRCODE).sleep();
        landOnQRcode(5.72-0.5, 1.44-1.5, 5);
        ros::Duration(TIME_TO_WAIT_ON_QRCODE).sleep();
        landOnQRcode(13.71-0.5, 4.91-1.5, 6);
        ros::Duration(TIME_TO_WAIT_ON_QRCODE).sleep();
	landOnPoint(STARTING_POSITION_X, STARTING_POSITION_Y);
      }
      else {
      	cout<<"Failed to activate the QR detector."<<endl;
      }
		}
	}
	cout<<"Bye bye :)"<<endl;
}

bool MASTER::exploration_phase(){
  //parameters to set waypoints
	float lenght_stride = 0.0;
	float lenght_x = 0.0;
	float lenght_y = 0.0;
	float size_uav = 0.0;
	/*reading parameters*/
	_nh.getParam("/master_exe/lenght_stride", lenght_stride);
	_nh.getParam("/master_exe/lenght_x", lenght_x); //_lenght_x:=20 _lenght_y:=10 _size_uav:=1 _maxvel:=20 _maxacc:=5 _lenght_stride:=8
	_nh.getParam("/master_exe/lenght_y", lenght_y);
	_nh.getParam("/master_exe/size_uav", size_uav);
	compute_waypoints(lenght_stride, lenght_x, lenght_y, size_uav);

	print_waypoints();

	cout<<"------> Type 's' to stop the exploration. "<<endl<<endl;
	_need_input = true;

	ros::Rate r(2);

	int current_waypoint = 0;
	px4_planner::planner_commander_service req;
	req.request.qr_code = -1;

	while(ros::ok() && (current_waypoint < _x_waypoints.size())){
		if (_new_key_input){
			_new_key_input = false;
			if (inputString=="s") {
				cout<<"Key input abort, exploration stopped. Going back to the base..."<<endl;
				landOnPoint(STARTING_POSITION_X, STARTING_POSITION_Y);
				return false;
			}
		}

		if(qr_code_finded >= NUM_QRCODE){
			cout<<"All QR code finded! Going back to the base..."<<endl;
			landOnPoint(STARTING_POSITION_X, STARTING_POSITION_Y);
			return true;
		}

		reachPoint(_x_waypoints[current_waypoint], _y_waypoints[current_waypoint]);
		current_waypoint++;
		r.sleep();
	}
	cout<<"Waypoints terminated. Going back to the base..."<<endl;
	landOnPoint(STARTING_POSITION_X, STARTING_POSITION_Y);
	cout<<"returning.."<<endl;
	return true;
}

bool MASTER::landOnQRcode(float x, float y, int qr_code){
	px4_planner::planner_commander_service req;
	req.request.x = x;	req.request.y = y;	req.request.qr_code = qr_code;	req.request.land_mask = 2;

	//req.request.land_mask = px4_planner_node::planner_commander_service::QR_CODE_LAND;

	cout<<" - Sending the mission to land on the QR code "<<qr_code<<" on ("<<x<<" "<<y<<")"<<endl;
	if (_client_planner.call(req)){
		if(req.response.completed){
			cout<<" -- Mission correctly completed"<<endl<<endl;
		}
		else {
			cout<<" -- Planner can't complete the mission"<<endl;
			return false;
		}
	}
	else {
		cout<<" - Not possible to connect with planner"<<endl;
		return false;
	}
	return true;
}

bool MASTER::landOnPoint(float x, float y){
	px4_planner::planner_commander_service req;
	req.request.x = x;
	req.request.y = y;
	req.request.qr_code = -1;
	req.request.land_mask = 1;

	cout<<" - Sending the mission to land on the point ("<<x<<" "<<y<<")"<<endl;
	if (_client_planner.call(req)){
		if(req.response.completed){
			cout<<" -- Mission correctly completed"<<endl<<endl;
		}
		else {
			cout<<" -- Planner can't complete the mission"<<endl;
			return false;
		}
	}
	else {
		cout<<" - Not possible to connect with planner"<<endl;
		return false;
	}
	return true;
}

bool MASTER::reachPoint(float x, float y){
	px4_planner::planner_commander_service req;
	req.request.x = x;
	req.request.y = y;
	req.request.qr_code = -1;
	req.request.land_mask = 0;

	cout<<" - Sending the mission to reach the point ("<<x<<" "<<y<<")"<<endl;
	if (_client_planner.call(req)){
		if(req.response.completed){
			cout<<" -- Mission correctly completed"<<endl<<endl;
		}
		else {
			cout<<" -- Planner can't complete the mission"<<endl;
			return false;
		}
	}
	else {
		cout<<" - Not possible to connect with planner"<<endl;
		return false;
	}
	return true;
}

void MASTER::compute_waypoints(const float& lenght_stride, const float& lenght_x, const float& lenght_y, const float& size_uav){

	std::cout<<"Parameters list: "<<std::endl;
	std::cout<<"lenght_y "<<lenght_y<<" lenght_stride: "<<lenght_stride<<" size_uav: "<<size_uav<<" lenght_stride: "<<lenght_stride<<std::endl;

	int n = (int)lenght_y/lenght_stride;
	if(n==0) {
		ROS_ERROR("too wide lenght_stride");
		return;
	}

	// generation points
  _x_waypoints.resize(n*2);
	_y_waypoints.resize(n*2);

	for(int i=0; i < n; i++){
		_x_waypoints(i*2) = lenght_stride/2;
		_x_waypoints(i*2+1) = lenght_x - lenght_stride/2;
		_y_waypoints(i*2+1) = _y_waypoints(i*2) = lenght_stride/2 + i*lenght_stride;
	}


  //set waypoints to be sure to perlustrate all the box
  if(n != lenght_y/lenght_stride){
 	  _x_waypoints.resize(n*2+2);
		_y_waypoints.resize(n*2+2);

		_x_waypoints(n*2) = lenght_stride/2;
		_x_waypoints(n*2+1) = lenght_x - lenght_stride/2;

		_y_waypoints(n*2+1) = _y_waypoints(n*2) = lenght_y - lenght_stride/2;
	}

	int j = 2;
	while(j < _x_waypoints.size()){
		double x_swap,y_swap;
		x_swap = _x_waypoints(j);
		y_swap = _y_waypoints(j);

		_x_waypoints(j) = _x_waypoints(j+1);
		_x_waypoints(j) = _x_waypoints(j+1);

		_x_waypoints(j+1) = x_swap;
		_y_waypoints(j+1) = y_swap;

		j = j+4;
	}



	//converting the way point from box frame to map frame
	for(int i=0;i<_x_waypoints.size();i++){
		_x_waypoints(i) = _x_waypoints(i) - 0.5;
		_y_waypoints(i) = _y_waypoints(i) - 1.5;
	}
}

void MASTER::print_waypoints(){
	std::cout<<endl<<"-----> These are the waypoints: "<<endl;
	for(int i=0;i<_x_waypoints.size();i++){
		std::cout << "x_waypoint "<<i<<":  "<<_x_waypoints(i);
		std::cout << "  y_waypoint "<<i<<":  "<<_y_waypoints(i) <<"\n";
	}
	cout<<endl;
}

void MASTER::run() {
	boost::thread key_input_t( &MASTER::key_input, this );
	boost::thread master_menu_t( &MASTER::master_menu, this );
	ros::spin();
}



int main( int argc, char** argv ) {
	ros::init(argc, argv, "master_node");
	MASTER master;
	master.run();
	return 0;
}

