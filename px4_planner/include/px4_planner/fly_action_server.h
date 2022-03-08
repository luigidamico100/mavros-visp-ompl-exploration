/*********************************************************************
The aim of this node is to implement an interface to allow the execution of exploration subtask (takeoff, landing, following path, 
landing on a QR code). The communication is performed via the "ROS Action Protocol", i.e. this node implement a SimpleActionServer. 
To execute a subtask will be necessary to send a goal with the correct bitmask, which encoding is specifed in the action definition file.

To avoid the stuck at the end of a "short path", caused by the time needed to communicate with the client and to construct the KDL trajectory 
object, the action server use a queue of KDL trajectory. 
Precisely when the action server receive the first FOLLOWPATH action goal waits the next goal before to start the path following. 
The purpose is to have in the queue, during the following of the current trajectory (corresponding to a "short path" element), 
the next ready to use KDL trajectory (corresponding to the next "short path" element), in order to let the PID controller fluenty track 
the "long path" composed by "short path" elements.  
*********************************************************************/



#ifndef FLY_ACTION_SERVER_H 
#define FLY_ACTION_SERVER_H


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <qr_detector_pkg/qr_detection_msg.h>

#include <visualization_msgs/Marker.h>

#include <px4_planner/FlyAction.h>

/*#include <kdl/velocityprofile_trap.hpp>*/
#include <kdl/velocityprofile_rect.hpp>
/*#include <kdl/velocityprofile_spline.hpp>*/

#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/path_roundedcomposite.hpp>

#include "ros/ros.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
/*#include "geometry_msgs/Vector3.h"*/
#include "geometry_msgs/TwistStamped.h"
#include <nav_msgs/Path.h>
#include "std_msgs/Float32.h"
//#include <termios.h>
#include "mavros_msgs/State.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandTOL.h"
#include <Eigen/Dense>

#include <actionlib/server/simple_action_server.h>

#include "qr_detector_pkg/qr_position_service.h"

#include <math.h>
#include <tf/tf.h>

#include <queue>    

#define PID_FREQ 10.0
#define FAULT_THRESHOLD 5.0
#define LANDING_ERROR_THRESHOLD 0.04
#define QR_LANDING_ERROR_THRESHOLD 0.025
/*COLORS*/
#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)


 
/********************************************************
Class to implement a PID controller.
*********************************************************/
class PID_contr {
  public:
   	PID_contr(int size){set_size(size);};
  	
  	PID_contr(float Kp, float Ki, float Kd1, float Kd2, float u_min, float u_max , Eigen::VectorXd y, Eigen::VectorXd y_des)  
  	:_Kp(Kp), _Ki(Ki), _Kd1(Kd1), _Kd2(Kd2),_u_max(u_max), _u_min(u_min){
  	 	if(y_des.size()!=y.size()) 
  	 		ROS_ERROR("PID error: y and y_des are not same size!");
	 		else{
  	 	set_size(y.size());
  	 	_e_old = y_des-y;
  	 	}
  	};
  	
   	void set_gains(float Kp, float Ki, float Kd1, float Kd2){_Kp = Kp; _Ki = Ki; _Kd1 = Kd1; _Kd2 = Kd2;}
   	void set_bounds(float u_min, float u_max){ _u_min = u_min; _u_max = u_max;}
   	void set_size(int size);
   	void reset_PID(){_u_d.setZero(); _u_i.setZero(); _e_old.setZero(); } 
   	
		Eigen::VectorXd ctrl_loop_PID(Eigen::VectorXd e); 
  private:
  	float _Kp; //gains
  	float _Ki;
  	float _Kd1;
  	float _Kd2;
 		float _u_max; //saturation boundaries control input
 		float _u_min;
 		Eigen::VectorXd _e_old;
  	Eigen::VectorXd _u_i;
  	Eigen::VectorXd _u_d; 	
  	Eigen::VectorXd e;
		Eigen::VectorXd u;
		Eigen::VectorXd u_i_new;

};


/********************************************************
Class that uses PID controllers to compute velocity command in order to follow path. The received path is used to generate KDL trajectories which are used to give the reference to the PID.
*********************************************************/

class PathFollower {
	
  public:
  	PathFollower(float freq):PID_pos(3), PID_yaw(1), _freq(freq){};//need to set gain
  	
  	float get_percentage_traveled(); //to give the feedback to client action server 
    int traj_queue_size(){return _traject_queue.size();}
    bool check_queue_empty(){return _traject_queue.empty();}
    bool check_disaster(Eigen::Vector4d curr_pose);
  	
    void push_traj( nav_msgs::Path path, float maxvel); //given a path generates the trajectory 
    Eigen::Vector4d compute_speed_command(Eigen::Vector4d curr_pose);
    void publishing_rviz_topic(const Eigen::Vector4d& curr_pose, const Eigen::Vector3d& des_pos, double des_yaw,const Eigen::Vector3d& lin_vel);
    
    void reset_PIDs_state(){ PID_pos.reset_PID(); PID_pos.reset_PID(); } //to call when a path is finished (e.g. go out the PIDs loop) 
    void set_gains_pos(float Kp, float Ki, float Kd1, float Kd2){PID_pos.set_gains(Kp, Ki, Kd1, Kd2); _gains_pos_setted = true;}
    void set_gains_yaw(float Kp, float Ki, float Kd1, float Kd2){PID_yaw.set_gains(Kp, Ki, Kd1, Kd2); _gains_yaw_setted = true;}
    void set_bounds_pos(float u_min, float u_max){PID_pos.set_bounds(u_min, u_max); _bounds_pos_setted = true;}
  	void set_bounds_yaw(float u_min, float u_max){PID_yaw.set_bounds(u_min, u_max); _bounds_yaw_setted = true;}
  	void set_rviz_publisher(ros::Publisher dp_pub, ros::Publisher mv_pub){_marker_velocity_pub = mv_pub; _des_pose_pub = dp_pub;};
  private:
  	Eigen::VectorXd yaw_control(float des_yaw, float yaw);
    //PIDs parameters
  	PID_contr PID_pos; //control (x,y,z) of the uav
  	PID_contr PID_yaw; //control yaw angle
   	bool _gains_pos_setted = false;
   	bool _gains_yaw_setted = false;
   	bool _bounds_pos_setted = false;
   	bool _bounds_yaw_setted = false;
 		//PathFollower parameters
  	float _freq;
  	float _time_current_traj = 0;
  	bool _curr_traj_finished = false;
		//kdl parameters
		std::queue<KDL::Trajectory*> _traject_queue;
		//rviz visualization parameters
		ros::Publisher _des_pose_pub;
		ros::Publisher _marker_velocity_pub;
};

/********************************************************

Class to implement the action server, also communicates with the UAV via mavros.
*********************************************************/

class FlyAction
{
		public:
			FlyAction(std::string name):
				_as(_nh, name, false),	_path_follower(10), _action_name(name), _pid_land(3) {initialize_server(); initialize_path_follower();}
		  
		  void run();	
		  
		 	void initialize_server();
		 	void initialize_path_follower();
		 	void arm_and_set_mode(const geometry_msgs::PoseStamped&);
			
		 	void px4_cntrl();
			void ctrl_loop();
		  void px4_PIDloop();
					 	
		  void mavros_state_cb( mavros_msgs::State mstate);
		  void mavros_local_pose_cb(geometry_msgs::PoseStamped);
			void qr_pos_cb(const qr_detector_pkg::qr_detection_msg&);
		  void goalCB();
			void preemptCB();
		protected:
			std::queue<px4_planner::FlyGoal> _goal_queue;
		
			void set_topics();
			
			bool _new_goal_available = false;
			bool _preempt_request = false;    
			bool disarmed = true;
			bool _new_qr_msg = false;	  
		  bool _ending_path = false;
		  bool _now_path_following = false;
		  
			
			ros::NodeHandle _nh;
			actionlib::SimpleActionServer<px4_planner::FlyAction> _as;
			std::string _action_name;
			px4_planner::FlyFeedback _feedback;
			px4_planner::FlyResult _result;
	 		px4_planner::FlyGoal _goal;
			px4_planner::FlyGoal _new_goal;
			 		
	 		PathFollower _path_follower;
			float _freq = PID_FREQ;
		  
			PID_contr _pid_land;
			ros::Subscriber _qr_pos_sub;
		  ros::Subscriber _qr_pos_base_sub;
		  qr_detector_pkg::qr_detection_msg _qr_pos;
		  
		  Eigen::Vector4d _local_pose;
		  mavros_msgs::State _mstate;
		  
		  ros::Publisher _des_pose_pub;	
			ros::Publisher _des_vel_pub;
		  ros::Publisher _local_pos_pub;
		  ros::Publisher _local_vel_pub;
	  	ros::Publisher _marker_velocity_pub;
		  
		  ros::Subscriber _mavros_state_sub;
		  ros::Subscriber _path_sub;
		  ros::Subscriber _local_pose_sub;
		  
		  ros::ServiceClient _arming_client;
		  ros::ServiceClient _set_mode_client;
		  ros::ServiceClient _land_client;
		  ros::ServiceClient _qr_pos_client;
	};

#endif 
