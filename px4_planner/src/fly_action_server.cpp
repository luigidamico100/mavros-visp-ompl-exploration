#include "px4_planner/fly_action_server.h"

/**********  PID_contr FUNCTIONS **********/

/**************************************************************
Initialize the dynamics vector used by the class
***************************************************************/
void PID_contr::set_size(int size){ 
	_e_old.resize(size); 
	_u_i.resize(size); 
	_u_d.resize(size); 
	_u_d.setZero(); 
	_u_i.setZero();	
	e.resize(size);
	_e_old.setZero(); 
	u.resize(size); 
	u_i_new.resize(size);
}

/**************************************************************
Implement the control loop, need to be called with fixed frequency 
***************************************************************/
Eigen::VectorXd PID_contr::ctrl_loop_PID(Eigen::VectorXd e){ //e = y_des-y
	bool update_u_i;
	
	u_i_new = _u_i+_Ki*e; 
	_u_d = _Kd1*_u_d+_Kd2*(e -_e_old); //if used need to set e_old for the first run
	u = _Kp*e+u_i_new+_u_d;
	for(int i=0;i < u.size(); i++){//anti wind up
		update_u_i = true;
		if(u(i) > _u_max){
			u(i) = _u_max;
			if(e(i) > 0) 
				update_u_i = false;
		} 
		else {
			if(u(i) < _u_min){
				u(i) = _u_min;
				if(e(i) < 0) 
					update_u_i = false;
			}
		}
		if(update_u_i) 
			_u_i(i) = u_i_new(i);
	}

	_e_old = e;
	return u; 
}


/**********  PathFollower FUNCTIONS **********/

/**************************************************************
param:
- path: path which points will be reached
- maxvel: velocity of the trajectory
- current_yaw: initial yaw of the trajectory (no longer considered)

This function will generate a trajectory considering the path and a rectangular velocity profile, and then the trajectory will be pushed in the queue  
***************************************************************/
void PathFollower::push_traj( nav_msgs::Path path, float maxvel){ 

	KDL::Path_RoundedComposite* path_kdl;
	
	std::cout<<"Pushing trajectory in the queue of size: "<<path.poses.size()<<std::endl;

	path_kdl = new KDL::Path_RoundedComposite(0.1,0.01,new KDL::RotationalInterpolation_SingleAxis());
	path_kdl->Add(KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(path.poses[0].pose.position.x,
									 																								path.poses[0].pose.position.y,
																															 	 	path.poses[0].pose.position.z)));
									
	for(int i=1;i < path.poses.size();i++){
		path_kdl->Add(KDL::Frame(KDL::Rotation::RPY(0,0,0), KDL::Vector(path.poses[i].pose.position.x,
							 																											path.poses[i].pose.position.y,
																																		path.poses[i].pose.position.z)));										
	}		
	path_kdl->Finish();
	
//		KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(maxvel,0.5); //trap_max_vel trap_max_acc
		KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Rectangular(maxvel); 

	velpref->SetProfile(0, path_kdl->PathLength());  
	_traject_queue.push(new KDL::Trajectory_Segment(path_kdl, velpref));
	

}

Eigen::Vector4d PathFollower::compute_speed_command(Eigen::Vector4d curr_pose){
	Eigen::Vector4d speed_command;
	speed_command.setZero();
	if(!_gains_pos_setted && !_gains_yaw_setted && !_bounds_pos_setted && !_bounds_yaw_setted){
		ROS_ERROR("Bounds or PID gains not setted yet!");
		return speed_command;
	}
	
	if(_curr_traj_finished){
		if(_traject_queue.empty()){
			ROS_ERROR("No other trajectory in queue!");
			return speed_command;
		}
		else{
			ROS_INFO("Starting trajectory of duration: %f",_traject_queue.front()->Duration());
			_time_current_traj = 0;
			_curr_traj_finished = false;
		}
	}
	else if(_traject_queue.empty())
		ROS_ERROR("Queue is empty, impossible to compute the speed!"); 	
	else if(_time_current_traj <= _traject_queue.front()->Duration()){
			//Preparing input for the PID
			
			//current x,y,z respectively
			Eigen::Vector3d position;
			position(0) = curr_pose(0); 
			position(1) = curr_pose(1);
			position(2) = curr_pose(2);
			//desired x,y,z respectively
			Eigen::Vector3d des_pos;
			des_pos(0) = _traject_queue.front()->Pos(_time_current_traj).p.x();
			des_pos(1) = _traject_queue.front()->Pos(_time_current_traj).p.y();
			des_pos(2) = _traject_queue.front()->Pos(_time_current_traj).p.z();				
			
			Eigen::Vector3d output_lin_vel;
			output_lin_vel = PID_pos.ctrl_loop_PID(des_pos-position);
			speed_command(0) = output_lin_vel(0) + _traject_queue.front()->Vel(_time_current_traj).vel.x(); 
			speed_command(1) = output_lin_vel(1) + _traject_queue.front()->Vel(_time_current_traj).vel.y();
			speed_command(2) = output_lin_vel(2) + _traject_queue.front()->Vel(_time_current_traj).vel.z();		
			//note: using kdl trajectory velocity for feedforward action
			
			//Computing desired yaw
			  
			float dy = (_time_current_traj+1/_freq > _traject_queue.front()->Duration())?
			_traject_queue.front()->Pos(_traject_queue.front()->Duration()).p.y()-_traject_queue.front()->Pos(_time_current_traj).p.y():
			_traject_queue.front()->Pos(_time_current_traj+1/_freq).p.y()-_traject_queue.front()->Pos(_time_current_traj).p.y();
			float dx = (_time_current_traj+1/_freq > _traject_queue.front()->Duration())?
			_traject_queue.front()->Pos(_traject_queue.front()->Duration()).p.x()-_traject_queue.front()->Pos(_time_current_traj).p.x():
			_traject_queue.front()->Pos(_time_current_traj+1/_freq).p.x()-_traject_queue.front()->Pos(_time_current_traj).p.x();
			
			double des_yaw = atan2(dy,dx);
			
			double yaw = curr_pose(3);
			speed_command(3) = yaw_control(des_yaw, yaw)(0);
				
			
			_time_current_traj += 1/_freq; // updating time for the next call
			
			if(_time_current_traj > _traject_queue.front()->Duration()){ 
				ROS_INFO("Trajectory finished");
				_curr_traj_finished = true;
				ROS_INFO("Eliminating finished trajectory from queue");
				if(_traject_queue.front()){ 
					delete _traject_queue.front(); // deletes aggregated path and profile instances, too
				}	
				_traject_queue.pop();
			}
			
			//publish topic for rviz DEBUG
			Eigen::Vector3d lin_vel;
			lin_vel(0) = speed_command(0); 
			lin_vel(1) = speed_command(1);
			lin_vel(2) = speed_command(2);
			publishing_rviz_topic(curr_pose, des_pos, des_yaw, lin_vel);

			return speed_command;		
		}
		
}


/**************************************************************
param:
- curr_pose: current position of the UAV 
This function is used to check if the UAV is very distant from the desired position, this situations happens only if a collision occurs.  
***************************************************************/
bool PathFollower::check_disaster(Eigen::Vector4d curr_pose){
	if(!_traject_queue.empty())
		if(  fabs(_traject_queue.front()->Pos(_time_current_traj).p.x() - curr_pose(0) > FAULT_THRESHOLD)
			|| fabs(_traject_queue.front()->Pos(_time_current_traj).p.y() - curr_pose(1) > FAULT_THRESHOLD)
			|| fabs(_traject_queue.front()->Pos(_time_current_traj).p.z() - curr_pose(2) > FAULT_THRESHOLD)){
	  	ROS_INFO("Disaster! current position: %f ; %f ; %f ", curr_pose(0), curr_pose(1), curr_pose(2));
	  	return true;
	  }
	return false;
}

/**************************************************************
This function publish topics that rviz will use to show the velocity command and the desired pose fo the UAV  
***************************************************************/
void PathFollower::publishing_rviz_topic(const Eigen::Vector4d& curr_pose, 
																				 const Eigen::Vector3d& des_pos, 
																				 double des_yaw,
																				 const Eigen::Vector3d& lin_vel){

			// publishing desired pose of the uav
			geometry_msgs::PoseStamped des_pose;
			des_pose.pose.position.x = des_pos(0);
			des_pose.pose.position.y = des_pos(1);
			des_pose.pose.position.z = des_pos(2);
			
			
			tf2::Quaternion des_quat;
			des_quat.setRPY( 0, 0, des_yaw );  // Create this quaternion from roll/pitch/yaw (in radians)
			des_quat.normalize();
			
			geometry_msgs::Quaternion des_quat_msg;
			des_quat_msg = tf2::toMsg(des_quat);
			
			des_pose.pose.orientation = des_quat_msg;
			
			des_pose.header.stamp = ros::Time::now();
			des_pose.header.frame_id = "map";
			
			_des_pose_pub.publish(des_pose);  
			
			// publishing velocity command marker
			visualization_msgs::Marker marker;
			marker.header.frame_id = "/map";
			marker.header.stamp = ros::Time::now();

			marker.ns = "basic_shapes";
			marker.id = 0;

			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;

			geometry_msgs::Point head , tail;
			tail.x = 0;
			tail.y = 0;
			tail.z = 0;
			marker.points.push_back(tail);
			head.x = lin_vel(0);
			head.y = lin_vel(1);
			head.z = lin_vel(2);
			marker.points.push_back(head);
			
			// Set the scale of the marker -- 1x1x1 here means 1m on a side
			marker.scale.x = 0.04;
			marker.scale.y = 0.1;
			marker.scale.z = 0.0;

			// Set the color -- be sure to set alpha to something non-zero!
			marker.color.r = 0.0f;
			marker.color.g = 1.0f;
			marker.color.b = 0.0f;
			marker.color.a = 1.0;

			marker.pose.position.x = curr_pose(0);
			marker.pose.position.y = curr_pose(1);
			marker.pose.position.z = curr_pose(2);
			
			marker.lifetime = ros::Duration(1/_freq);
			_marker_velocity_pub.publish(marker); 

}


Eigen::VectorXd PathFollower::yaw_control(float des_yaw, float yaw){
	Eigen::VectorXd yaw_input;
	yaw_input.resize(1);
  yaw_input(0) = des_yaw - yaw;
          
  if(fabs(yaw_input(0)) > M_PI)
    yaw_input(0) = yaw_input(0) - 2*M_PI* ((yaw_input(0)>0)?1:-1);

	return PID_yaw.ctrl_loop_PID(yaw_input);
}

float PathFollower::get_percentage_traveled(){
	if(!_curr_traj_finished)
		return _time_current_traj/_traject_queue.front()->Duration();
	else //ptr to Trajectory not exist anymore, runtime error if try to use it
		return 0;
}

/**********  FlyAction FUNCTIONS **********/


void FlyAction::initialize_server(){
	  
  //register the goal and feeback callbacks
  _as.registerGoalCallback(boost::bind(&FlyAction::goalCB, this));
  _as.registerPreemptCallback(boost::bind(&FlyAction::preemptCB, this));
  
	_arming_client = _nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	_set_mode_client = _nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	_local_pos_pub = _nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 0);
	_land_client = _nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
	_mavros_state_sub = _nh.subscribe("/mavros/state", 0, &FlyAction::mavros_state_cb, this);
	_local_pose_sub = _nh.subscribe("/mavros/local_position/pose", 0, &FlyAction::mavros_local_pose_cb, this);
	_local_vel_pub = _nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 0);
  
  _qr_pos_client = _nh.serviceClient<qr_detector_pkg::qr_position_service>("/qr_detector/qr_code_pos_map");
 	
	_des_pose_pub = _nh.advertise<geometry_msgs::PoseStamped>("fly_action_server/des_pose", 0);	
	_marker_velocity_pub = _nh.advertise<visualization_msgs::Marker>("fly_action_server/vel_cmd", 1);
	  
  _as.start();
}

void FlyAction::initialize_path_follower(){
	_local_pose.setZero();
	
	//get PID parameters
	float Kp_pos = 0.0; 
	float Ki_pos = 0.0;
	float Kd1_pos = 0.0;
	float Kd2_pos = 0.0;
	float u_max_pos = 0.0;
	float u_min_pos = 0.0; 	
	_nh.getParam("/fly_action_server/Kp_pos", Kp_pos);
	_nh.getParam("/fly_action_server/Ki_pos", Ki_pos);
	_nh.getParam("/fly_action_server/Kd1_pos", Kd1_pos);
	_nh.getParam("/fly_action_server/Kd2_pos", Kd2_pos);
	_nh.getParam("/fly_action_server/PID_u_max_pos", u_max_pos);
	_nh.getParam("/fly_action_server/PID_u_min_pos", u_min_pos);


	float Kp_yaw = 0.0; 
	float Ki_yaw = 0.0;
	float Kd1_yaw = 0.0;
	float Kd2_yaw = 0.0;
	float u_max_yaw = 0.0;
	float u_min_yaw = 0.0; 	
	_nh.getParam("/fly_action_server/Kp_yaw", Kp_yaw);
	_nh.getParam("/fly_action_server/Ki_yaw", Ki_yaw);
	_nh.getParam("/fly_action_server/Kd1_yaw", Kd1_yaw);
	_nh.getParam("/fly_action_server/Kd2_yaw", Kd2_yaw);
	_nh.getParam("/fly_action_server/PID_u_max_yaw", u_max_yaw);
	_nh.getParam("/fly_action_server/PID_u_min_yaw", u_min_yaw);

	_path_follower.set_gains_pos(Kp_pos, Ki_pos, Kd1_pos, Kd2_pos);
	_path_follower.set_gains_yaw(Kp_yaw, Ki_yaw, Kd1_yaw, Kd2_yaw);
	_path_follower.set_bounds_pos(u_min_pos, u_max_pos);
	_path_follower.set_bounds_yaw(u_min_yaw, u_max_yaw);


 //set publisher for rviz
	_path_follower.set_rviz_publisher(_des_pose_pub, _marker_velocity_pub);


//set PID parameters for landing 
	float Kp_land = 0.0; 
	float Ki_land = 0.0;
	float Kd1_land = 0.0;
	float Kd2_land = 0.0;
	float u_max_land = 0.0;
	float u_min_land = 0.0; 	
	_nh.getParam("/fly_action_server/Kp_land", Kp_land);
	_nh.getParam("/fly_action_server/Ki_land", Ki_land);
	_nh.getParam("/fly_action_server/Kd1_land", Kd1_land);
	_nh.getParam("/fly_action_server/Kd2_land", Kd2_land);
	_nh.getParam("/fly_action_server/PID_u_max_land", u_max_land);
	_nh.getParam("/fly_action_server/PID_u_min_land", u_min_land);	
	
	_pid_land.set_gains(Kp_land, Ki_land, Kd1_land, Kd2_land);
	_pid_land.set_bounds(u_min_land, u_max_land);	
}


void FlyAction::mavros_state_cb( mavros_msgs::State mstate)	{
	_mstate = mstate;
}


void FlyAction::mavros_local_pose_cb(geometry_msgs::PoseStamped p) {
	_local_pose(0) = p.pose.position.x; //local position (x,y,z) uav in box frame
	_local_pose(1) = p.pose.position.y;
	_local_pose(2) = p.pose.position.z; 
	
	// get yaw angle from quaternion		
	tf::Quaternion q(
  p.pose.orientation.x,
  p.pose.orientation.y,
  p.pose.orientation.z,
  p.pose.orientation.w);

	double roll, pitch;
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, _local_pose(3));		
}

void FlyAction::qr_pos_cb(const qr_detector_pkg::qr_detection_msg& qr_pos_msg){
	_qr_pos = qr_pos_msg;
//	ROS_INFO("QR position arrived: (%f , %f , %f) ",_qr_pos.qr_position.x, _qr_pos.qr_position.y, _qr_pos.qr_position.z);
	_new_qr_msg = true;		
}

void FlyAction::arm_and_set_mode(const geometry_msgs::PoseStamped& des_pos){
	// SET MODE AND TAKEOFF
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
			
	//send a few setpoints before starting		
	ros::Rate r(10);
  for(int i = 10; ros::ok() && i > 0; --i){
    _local_pos_pub.publish(des_pos);
	  r.sleep();
  }
			
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
	  ROS_INFO("Manual mode enabled");
	}
	if( _arming_client.call(arm_cmd) && arm_cmd.response.success){
	}
	while(!_mstate.armed ) usleep(0.1*1e6);
	ROS_INFO("Vehicle armed");
	if( _set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
	  ROS_INFO("Manual mode enabled");
	}
	disarmed = false;
}

/**************************************************************
The new goal will be processed by this function that take care to do all the operation needed.

Note: the feedback are not sent in most kind of goals beacause the client actually don't read them, the only information needed by the client is the completion of the goal.  
***************************************************************/
void FlyAction::px4_cntrl(){		
	ros::Rate r(10);
	geometry_msgs::PoseStamped des_pos;
	int index = 0;
	while(ros::ok()){
		if(!_new_goal_available){
			if(!disarmed && !_now_path_following){ //to avoid fail safe mode
				ROS_INFO("px4_cntrl(): Hovering, Waiting for a new goal");
				des_pos.pose.position.x = _local_pose(0); 
				des_pos.pose.position.y = _local_pose(1);
				des_pos.pose.position.z = _local_pose(2);				

				float hovering_time = 0;
				while(!_new_goal_available){ 
					hovering_time += 1/10;
					_local_pos_pub.publish(des_pos);
					r.sleep();
					if(hovering_time > 10){ //if hovering for more than 10 seconds, UAV will land
						disarmed = true;
						ROS_ERROR("No goal from action client, landing and disarm!");
						mavros_msgs::CommandTOL land_srv;
						_land_client.call( land_srv );
					}
				}
			}
			else{ //i.e. uav landed and waiting new goal 
				if(index == 20){ //publish message once on 20 times 
					ROS_INFO("px4_cntrl(): uav disarmed, waiting for takeoff command");
					index = 0;
				}
				index++; 
				r.sleep();
			}  
		}
		else{
			ROS_INFO("px4_cntrl(): Processing new goal");
			_new_goal_available = false;
			_preempt_request = false;
			const px4_planner::FlyGoal goal = _new_goal;
			if(_as.isPreemptRequested()){
				ROS_INFO("New goal has been preempted");
				ROS_INFO("%s: Preempted", _action_name.c_str());
				_as.setPreempted();
			}
			else 			
				switch (goal.command_mask)
				{
				case px4_planner::FlyGoal::TAKEOFF:
				{
					ROS_INFO("Takeoff action");
					if(!disarmed){
						ROS_WARN("Uav is armed! Already flying");
						_result.result = false;
						_as.setAborted(_result);
					}
					else{  
						des_pos.pose.position.x = _local_pose(0); 
						des_pos.pose.position.y = _local_pose(1);
						des_pos.pose.position.z = goal.path.poses[0].pose.position.z;					
						ROS_INFO("Takeoff setpoint: (%f, %f, %f) ", des_pos.pose.position.x, 
																												des_pos.pose.position.y,
																					 							des_pos.pose.position.z);
						arm_and_set_mode(des_pos);
						_feedback.progress = 0;
						_as.publishFeedback(_feedback);
						
						while( ros::ok() 
								&& (!_preempt_request) 
								&& (!( fabs(des_pos.pose.position.z - _local_pose(2)) < 0.07 ))) {
								
							_local_pos_pub.publish(des_pos);
							_feedback.progress = (int)(100*_local_pose(2)/des_pos.pose.position.z);
							_as.publishFeedback(_feedback);
							
							//ROS_INFO("des_pos - local_pose: %f", fabs(des_pos.pose.position.z - _local_pose(2)));
	//						ROS_INFO("Feedback: %d", _feedback.progress);
							r.sleep();
						}
						
						
						if(_preempt_request){
							_preempt_request = false;
							ROS_INFO("Takeoff has been preempted");
							ROS_INFO("%s: Preempted", _action_name.c_str());
							_as.setPreempted();
						}
						else{
					    ROS_INFO("%s: Succeeded", _action_name.c_str());
					    _result.result = true;
					    _as.setSucceeded(_result);
						
							//waits for a new goal in order to avoid to go in hovering if no goal still available when the 
							//takeoff is finished (caused by the time to compute the path for the client)
							while( ros::ok() 
								  	&& (!_new_goal_available)) {

								_local_pos_pub.publish(des_pos);
								r.sleep();
							}
						}

					}
					break;			
				}// end FlyGoal::TAKEOFF
				case px4_planner::FlyGoal::LAND: //used mostly to land at home
				{
					ROS_INFO("Land action");
					if(disarmed){
						ROS_WARN("Uav is not armed! Already on the ground");
						_result.result = false;
						_as.setAborted(_result);
					}
					else{	
						//desired land position chosen by the client (the x and y should coincide to the last position of the path in order to avoid obstacles)
						des_pos.pose.position.x = goal.path.poses[0].pose.position.x; 
						des_pos.pose.position.y = goal.path.poses[0].pose.position.y;
						des_pos.pose.position.z = _local_pose(2);		
						geometry_msgs::TwistStamped cmd_vel;
						Eigen::Vector3d cmd_vel_PID;
						Eigen::Vector3d land_pos;
						land_pos(0) = des_pos.pose.position.x;
						land_pos(1) = des_pos.pose.position.y;
						land_pos(2) = des_pos.pose.position.z;
						float initial_yaw = _local_pose(3);
						while( ros::ok() 
									&& (!_preempt_request)
							    && (!(fabs(land_pos(0) - _local_pose(0)) < LANDING_ERROR_THRESHOLD)
									|| !(fabs(land_pos(1) - _local_pose(1)) < LANDING_ERROR_THRESHOLD )) )
					  {
			  			Eigen::Vector3d local_pos;
			  			local_pos(0) = _local_pose(0);
			  			local_pos(1) = _local_pose(1);
			  			local_pos(2) = _local_pose(2);
			  			
							cmd_vel_PID = _pid_land.ctrl_loop_PID(land_pos-local_pos);
							cmd_vel.twist.linear.x = cmd_vel_PID(0);
							cmd_vel.twist.linear.y = cmd_vel_PID(1);
							cmd_vel.twist.linear.z = cmd_vel_PID(2);
							
							//publishing for rviz visualization
							_path_follower.publishing_rviz_topic( _local_pose, land_pos, initial_yaw, cmd_vel_PID);

							_local_vel_pub.publish(cmd_vel);							
							r.sleep();
						}
						mavros_msgs::CommandTOL land_srv;
						_land_client.call( land_srv );
						while(ros::ok() && _mstate.armed);
						disarmed = true;
						ROS_INFO("%s: Succeeded", _action_name.c_str());
				    _result.result = true;
				    _as.setSucceeded(_result);
					} 
					break;
				}// end FlyGoal::LAND 
				case px4_planner::FlyGoal::QrCODE_LAND:
				{
					ROS_INFO("QR Code land action");
					if(disarmed){
						ROS_WARN("Uav is not armed! Already on the ground");
						_result.result = false;
						_as.setAborted(_result);
					}
					else{ 
						//after the reaching of the QR code position specified by the action client, it will read the position from this topic in order
						//to not be affected by outliers during the misuration of the QR code position done during the exploration phase   
						_qr_pos_sub = _nh.subscribe("/qr_detector/qr_pos_mov_average", 1, &FlyAction::qr_pos_cb, this);
						
						/*wait in hovering for the first qr position callback*/

						Eigen::Vector3d initial_pos;
						initial_pos(0) = _local_pose(0);
						initial_pos(1) = _local_pose(1);
						initial_pos(2) = _local_pose(2);
						geometry_msgs::TwistStamped cmd_vel;
						Eigen::Vector3d cmd_vel_PID;
						Eigen::Vector3d qr_land_pos;
						_pid_land.reset_PID();
						bool first_msg_arrived = false;	
						float initial_yaw = _local_pose(3);
						float t = 0;		
					  while( ros::ok() 
									&& (!_preempt_request)
									&&  (!first_msg_arrived)
									&& (t < 10))
					  {
			  			Eigen::Vector3d local_pos;
			  			local_pos(0) = _local_pose(0);
			  			local_pos(1) = _local_pose(1);
			  			local_pos(2) = _local_pose(2);
			  			
							cmd_vel_PID = _pid_land.ctrl_loop_PID(initial_pos-local_pos);
							cmd_vel.twist.linear.x = cmd_vel_PID(0);
							cmd_vel.twist.linear.y = cmd_vel_PID(1);
							cmd_vel.twist.linear.z = cmd_vel_PID(2);
							
							//publishing for rviz visualization 							
							_path_follower.publishing_rviz_topic( _local_pose, initial_pos, initial_yaw, cmd_vel_PID);
							_local_vel_pub.publish(cmd_vel);							
							
							if(_new_qr_msg && (_qr_pos.qr_code == goal.qr_code)){ //check if could stop to hovering
								qr_land_pos(0) = _qr_pos.qr_position.x;
								qr_land_pos(1) = _qr_pos.qr_position.y;
								qr_land_pos(2) = _local_pose(2);
								first_msg_arrived = true;
							}
							t += 1/10;
							r.sleep();
						}				
						_new_qr_msg = false;
						
						if(t < 10)
							/* reaching precise QR code position*/
							while( ros::ok() 
										&& (!_preempt_request)
									  && (!(pow(qr_land_pos(0) - _local_pose(0),2) + pow(qr_land_pos(1) - _local_pose(1),2) <
									  		 pow(QR_LANDING_ERROR_THRESHOLD,2))) )
							{
								if(_new_qr_msg && (_qr_pos.qr_code == goal.qr_code)){ //updating QR code position
									qr_land_pos(0) = _qr_pos.qr_position.x;
									qr_land_pos(1) = _qr_pos.qr_position.y;
									_new_qr_msg = false;
								}

								Eigen::Vector3d local_pos;
								local_pos(0) = _local_pose(0);
								local_pos(1) = _local_pose(1);
								local_pos(2) = _local_pose(2);
								
								cmd_vel_PID = _pid_land.ctrl_loop_PID(qr_land_pos-local_pos);
								cmd_vel.twist.linear.x = cmd_vel_PID(0);
								cmd_vel.twist.linear.y = cmd_vel_PID(1);
								cmd_vel.twist.linear.z = cmd_vel_PID(2);
								
								//publishing for rviz visualization 							
								_path_follower.publishing_rviz_topic( _local_pose, initial_pos, initial_yaw, cmd_vel_PID);

								_local_vel_pub.publish(cmd_vel);							
								r.sleep();
							}
						else{
							ROS_WARN("QR code not finded, timer expired");
						_result.result = false;
						_as.setAborted(_result);
						}
						
						if(_preempt_request && (t < 10)){
							_preempt_request = false;
							ROS_INFO("%s: Preempted", _action_name.c_str());
							_as.setPreempted();
						}
						else{
							mavros_msgs::CommandTOL land_srv;
							_land_client.call( land_srv );
							while(ros::ok() && _mstate.armed);
							disarmed = true;
							ROS_INFO("%s: Succeeded", _action_name.c_str());
							_result.result = true;
							_as.setSucceeded(_result);
					  }			
					}
					break;
				}// end FlyGoal::QrCODE_LAND 
			
				case px4_planner::FlyGoal::FOLLOWPATH:
				{
					if(disarmed){
						ROS_WARN("Uav is not armed! Need to takeoff");
						_result.result = false;
						_as.setAborted(_result);
					}
					else{
						ROS_INFO("px4_cntrl(): Following the path action, path velocity: %f", goal.path_vel);
						for(int i=0;i<goal.path.poses.size();i++){
						ROS_INFO("Goal checkpoint path received  ( %f , %f , %f )", goal.path.poses[i].pose.position.x,
																																				goal.path.poses[i].pose.position.y,
																																				goal.path.poses[i].pose.position.z);
																																				
						}
						_path_follower.push_traj(goal.path, goal.path_vel);
						std::cout<<"px4_cntrl(): Received new checkpoint of the path, total number of checkpoint: "<<
																																			_path_follower.traj_queue_size()<<std::endl;
						
						int initial_size = _path_follower.traj_queue_size();
						if(initial_size > 1) //if initial_size = 1 set immediately goal succeded because is waiting for the next goal to start pathfollowing
							while(ros::ok() 
									&& (_path_follower.traj_queue_size() != initial_size-1 )//waits current trajectory is finished
									&& !_preempt_request )
								r.sleep(); //waiting initial_size become lower
						else 
							ROS_INFO("px4_cntrl(): First checkpoint received, intiating path! ");
						
						if(_preempt_request){
							_preempt_request = false;
							ROS_INFO("%s: Preempted", _action_name.c_str());
							_as.setPreempted();
						}
						else{
							ROS_INFO("px4_cntrl(): FOLLOWPATH Action completed");
							_result.result = true;
							_as.setSucceeded(_result);
					  }
					}
					break;
				}// end FlyGoal::FOLLOWPATH 
				case px4_planner::FlyGoal::END_PATH: //this goal is received to notify that the path following is finished 
				{
					ROS_INFO("End trajectory action");
					_ending_path = true;
					ROS_INFO("Waiting ending last checkpoint, trajectory queue size: %d",_path_follower.traj_queue_size());
					while(ros::ok() 
								&& !_path_follower.check_queue_empty() //waits that the queue be empty, i.e. finish all the trajectories 
								&& !_preempt_request )
						r.sleep();
					_ending_path = false;
					if(_preempt_request){
						_preempt_request = false;
						ROS_INFO("%s: Preempted", _action_name.c_str());
						_as.setPreempted();
					}
					else{
						std::cout<<"End trajectory"<<std::endl;
						_result.result = true;
						_as.setSucceeded(_result);
					}
					
					break;
				}
				default:
					ROS_ERROR("Invalid command mask");
				break;
			}//end switch		
		}
	}
}

void FlyAction::goalCB()
{
	ROS_INFO("goalCB(): New goal available");
	_preempt_request = false; 
	_new_goal = *_as.acceptNewGoal();
	
	if(_as.isPreemptRequested()){
		ROS_INFO("New goal has been preempted");
		ROS_INFO("%s: Preempted", _action_name.c_str());
		_as.setPreempted();
	}
	else{ // check if the new goal is consistent considering the path following action
		if(!_path_follower.check_queue_empty()
				&& (!_new_goal.command_mask == px4_planner::FlyGoal::FOLLOWPATH
				|| !_new_goal.command_mask == px4_planner::FlyGoal::END_PATH))
		{
			ROS_ERROR("Request action different from 'END_PATH' or 'FOLLOWPATH' action while path following");
			_as.setAborted();
		}
		else{
			std::cout<<"goalCB(): Goal accepted, goal code: "<< unsigned(_new_goal.command_mask)<<std::endl;
			_new_goal_available = true;
		}
	}
}

void FlyAction::preemptCB()
{
	ROS_INFO("Preempt request");		
	_preempt_request = true;
	
}

void FlyAction::px4_PIDloop(){
	geometry_msgs::TwistStamped cmd_vel;
	Eigen::Vector4d cmd_vel_PID;
	ros::Rate r(PID_FREQ);
	while( ros::ok() ){
		while( (_path_follower.traj_queue_size() < 2) && !_ending_path)
			r.sleep();
		_now_path_following = true;
		ROS_INFO("px4_PIDloop(): Start following path");
		
		while( !_path_follower.check_queue_empty() && !_path_follower.check_disaster(_local_pose) ) {
			cmd_vel_PID = _path_follower.compute_speed_command(_local_pose);
			cmd_vel.twist.linear.x = cmd_vel_PID(0);
			cmd_vel.twist.linear.y = cmd_vel_PID(1);
			cmd_vel.twist.linear.z = cmd_vel_PID(2);
			
			cmd_vel.twist.angular.z = cmd_vel_PID(3);
			_local_vel_pub.publish(cmd_vel);							
			r.sleep();
		}

		cmd_vel.twist.linear.x = 0;
		cmd_vel.twist.linear.y = 0;
		cmd_vel.twist.linear.z = 0;
		
		cmd_vel.twist.angular.z = 0;
		_local_vel_pub.publish(cmd_vel);	

		_now_path_following = false;
		//at the next path we start with resetted integrator
		_path_follower.reset_PIDs_state(); 
		if(_path_follower.check_disaster(_local_pose)){ 
			disarmed = true;
			ROS_ERROR("Error too much bigger! Landing and disarm");
			mavros_msgs::CommandTOL land_srv;
			_land_client.call( land_srv );
		}
		r.sleep();
	}
}

void FlyAction::run(){	
	boost::thread px4_cntrl_t( &FlyAction::px4_cntrl, this );
	boost::thread px4_PIDloop_t( &FlyAction::px4_PIDloop, this );	
	ros::spin();
}



