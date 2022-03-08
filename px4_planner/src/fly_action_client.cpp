/*********************************************************************
The aim of this node is to receive a "long path" (in the x and y coordinates) from the master node and
split into more "shorter path" (in the x and y coordinates). The computed paths are sended to a node that directly commands the UAV.
Once a path are sended, the node waits that it is performed by the UAV and compute the next path using an RTT star planning algorithm.
Given a path in x and y coordinates, the planning alghoritm is used to obtain the z coordinates needed to avoid obtacles.
The obstacles are detected using an octomap that is builded during the UAV movements.
The solution to plan more "shorter path" is adopted because the entire octomap of the environmnet is unknown and has to be builded.
*********************************************************************/


#include "ros/ros.h"
#include <Eigen/Dense>
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <cmath>

#include "boost/thread.hpp"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <ompl/config.h>
#include <ompl/util/Exception.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <px4_planner/FlyAction.h>

//#include "rosservice/service.h"
#include <px4_planner/planner_commander_service.h>


#include "fcl/config.h"
#include "fcl/octree.h"
#include "fcl/collision.h"
#include "fcl/broadphase/broadphase.h"
#include <octomap_msgs/conversions.h>


/* Solver variables */
#define OCTOMAP_TOPICNAME "/octomap_binary"
#define SOLVE_PLANNING_TIME_LIMIT 1
#define BOX_SIZE_1 1.2
#define BOX_SIZE_2 1.2
#define BOX_SIZE_3 1.2
#define STARTING_HEIGHT 1.7
#define INCREASE_HEIGHT 0.5 //height increase amount when the planning fails.
#define NPATH_BEFORE_DECREASE_ALTITUDE 2

/* Client variables */
#define START_PATH_PLANNING 6
#define ACTIONCLIENTSERVER_NAME "PathCommand_actionserver"

#define CHECKPOINTS_MAXDISTANCE 1.5 //how to compute checkpoints?
#define PATH_VEL_MAX 1.0
#define PATH_VEL_MIN 0.5

#define CLIENTSERVER_NAME "planner_commander_service"


using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;



/*************************************************************************************************************************************************
Class to manage the points used to feed the planning algorithm. The points are called checkpoints.
x_checkpoints: contains the x coordinates of checkpoints
y_checkpoints: contains the x coordinates of checkpoints
points_number: number of points
index: public attribute, contains the index of the current check point. The class works like an iterator
*************************************************************************************************************************************************/

class POINTS_INTERATOR {
  public:
    POINTS_INTERATOR();
    void create_checkPoints(float x_i, float x_f, float y_i, float y_f, float max_distance);
    bool haveMorePoints();

    float get_current_x_checkpoint();
    float get_current_y_checkpoint();
    void print_checkpoints();
    int index;
    int points_number;

  private:
    Eigen::VectorXd x_checkpoints;
    Eigen::VectorXd y_checkpoints;

};


/*************************************************************************************************************************************************
Class Constructor.
*************************************************************************************************************************************************/
POINTS_INTERATOR::POINTS_INTERATOR() {
  index = 0;
  points_number = 0;
}

/*************************************************************************************************************************************************
Given an initial point, a final points (in x and y) and a maximum distance, compute the intermediate equidistant checkpoints (in x and y)
*************************************************************************************************************************************************/
void POINTS_INTERATOR::create_checkPoints(float x_i, float y_i, float x_f, float y_f, float max_distance) {

  index = 0;
  float path_distance = sqrt(pow(x_f - x_i,2) + pow(y_f-y_i,2));

  points_number = ceil(path_distance/max_distance)+1;

  x_checkpoints.resize(points_number);
  y_checkpoints.resize(points_number);

  float x_distance = x_f - x_i, y_distance = y_f - y_i;
  for (int i=0; i<points_number; i++) {
    //cout<<"Creating"<< i<<": "<<y_i + y_distance/points_number * i<<endl;
    x_checkpoints(i) = x_i + x_distance/(points_number-1) * i;
    y_checkpoints(i) = y_i + y_distance/(points_number-1) * i;
  }
  if (x_i == x_f && y_i == y_f) {
      x_checkpoints(0) = x_f;
      y_checkpoints(0) = y_f;
  }

}

/*************************************************************************************************************************************************
Used to know if the class has more points to iterate
*************************************************************************************************************************************************/
bool POINTS_INTERATOR::haveMorePoints() {
  if (index >= (points_number-1))
    return false;
  return true;
}

/*************************************************************************************************************************************************
Get the x-coordinate of the current checkpoint
*************************************************************************************************************************************************/
float POINTS_INTERATOR::get_current_x_checkpoint() {
  return x_checkpoints(index);
}
/*************************************************************************************************************************************************
Get the y-coordinate of the current checkpoint
*************************************************************************************************************************************************/
float POINTS_INTERATOR::get_current_y_checkpoint() {
   return y_checkpoints(index);
 }

/*************************************************************************************************************************************************
Print the checkpoints computed
*************************************************************************************************************************************************/
void POINTS_INTERATOR::print_checkpoints() {
  std::cout<<"\n\n These are the checkpoints!: \n"<<endl;
  for(int i=0;i<points_number;i++){
  		std::cout << "x_checkpoints "<<i<<":  "<<x_checkpoints(i);
  		std::cout << "  y_checkpoints "<<i<<":  "<<y_checkpoints(i) <<"\n";
  	}
}


/************************************************************************************************************************************************
*************************************************************************************************************************************************
*************************************************************************************************************************************************
*************************************************************************************************************************************************
************************************************************************************************************************************************/



/*************************************************************************************************************************************************
Class to compute the planning. It uses an RTT star algorithm.
*************************************************************************************************************************************************/
class OMPL_PLAN {
	public:
		OMPL_PLAN();
		nav_msgs::Path run(float x_i,float y_i, float z_i, float x_f, float y_f, float z_f);
		void ompl_init(float x_i,float y_i, float z_i, float x_f, float y_f, float z_f);
		void plan();
		ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si);
  	bool isStateValid(const ob::State *state);
		bool isStateValid2(const ob::State *state);
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg);
		void updateMap(std::shared_ptr<fcl::CollisionGeometry> map);

	private:
		ob::StateSpacePtr _space;
		ob::SpaceInformationPtr _si;
		ob::ProblemDefinitionPtr _pdef;
		ros::NodeHandle _nh;
    std::shared_ptr<fcl::CollisionGeometry> _Robot;
	  std::shared_ptr<fcl::CollisionGeometry> _tree_obj;
	  ros::Subscriber _octree_sub;
	  bool solving_planning = false;
	  nav_msgs::Path *generated_path = NULL;
};

OMPL_PLAN::OMPL_PLAN() {
  _Robot = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(BOX_SIZE_1, BOX_SIZE_2, BOX_SIZE_3));
  fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.05)));
  _tree_obj = std::shared_ptr<fcl::CollisionGeometry>(tree);
  _octree_sub = _nh.subscribe<octomap_msgs::Octomap>(OCTOMAP_TOPICNAME, 1, &OMPL_PLAN::octomapCallback, this);
}

void OMPL_PLAN::updateMap(std::shared_ptr<fcl::CollisionGeometry> map) {
	_tree_obj = map;
  cout<<"-----> Octomap Update"<<endl;
}

void OMPL_PLAN::octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg) {

  cout<<"-----> Octo callback"<<endl;

  	// convert octree to collision object
  	octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
  	fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));

    //ros::Rate r(5);
    while (ros::ok() && solving_planning) {
      cout<<"----> Waiting to update octomap"<<endl;
      //r.sleep();
    }
  	// Update the octree used for collision checking
  	updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
}



/*************************************************************************************************************************************************
Check if a state is valid accordingly to the published octomap
*************************************************************************************************************************************************/
bool OMPL_PLAN::isStateValid(const ob::State *state){
	// cast the abstract state type to the type we expect
	const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

	// extract the first component of the state and cast it to what we expect
	const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

	// extract the second component of the state and cast it to what we expect
	const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

	//cout << "Check " << pos->values[0] << " " << pos->values[1] << " " << pos->values[2] << endl;

	fcl::CollisionObject treeObj((_tree_obj));
	fcl::CollisionObject robotObject(_Robot);

	// check validity of state defined by pos & rot
	fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
	fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
	robotObject.setTransform(rotation, translation);
	fcl::CollisionRequest requestType(1,false,1,false);
	fcl::CollisionResult collisionResult;
	fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

	return(!collisionResult.isCollision());

}

bool OMPL_PLAN::isStateValid2(const ob::State *state) {
  return true;
}

ob::OptimizationObjectivePtr OMPL_PLAN::getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si) {
	ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
	obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	return obj;
}

/*************************************************************************************************************************************************
Inizialize the planner
*************************************************************************************************************************************************/
void OMPL_PLAN::ompl_init(float x_i,float y_i, float z_i, float x_f, float y_f, float z_f) {

	_space = ob::StateSpacePtr(new ob::SE3StateSpace());
  ob::ScopedState<ob::SE3StateSpace> start(_space);
  ob::ScopedState<ob::SE3StateSpace> goal(_space);
	ob::RealVectorBounds bounds(3);
	bounds.setLow(0,-20);
	bounds.setHigh(0,20); //x
	bounds.setLow(1,-20);
	bounds.setHigh(1,20); //y
	bounds.setLow(2,-20000);
	bounds.setHigh(2,20000); //z
/*  bounds.setLow(0,-1); //qui devono andare le coordinate del box parametrizzate
	bounds.setHigh(0,20); //x
	bounds.setLow(1,-1);
	bounds.setHigh(1,10); //y
	bounds.setLow(2, 0);
	bounds.setHigh(2,3.5); //z
*/
	_space->as<ob::SE3StateSpace>()->setBounds(bounds);

	// construct an instance of  space information from this state space
	_si = ob::SpaceInformationPtr(new ob::SpaceInformation(_space));


	goal->setXYZ(x_f,y_f,z_f);
	goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

  start->setXYZ(x_i,y_i,z_i);
	start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();


	_si->setStateValidityChecker(std::bind(&OMPL_PLAN::isStateValid, this, std::placeholders::_1 ));

	_pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(_si));
	_pdef->setStartAndGoalStates(start, goal);
	_pdef->setOptimizationObjective(OMPL_PLAN::getPathLengthObjWithCostToGo(_si));


	std::cout << "Planner initialized!" << std::endl;
}


/*************************************************************************************************************************************************
Plan function
*************************************************************************************************************************************************/
void OMPL_PLAN::plan() {

	//cout << "Press enter to plan!" << endl;
	//string line;
	//getline(cin, line);

	// create a planner for the defined space
	ob::PlannerPtr plan(new og::RRTstar(_si));

	// set the problem we are trying to solve for the planner
	plan->setProblemDefinition(_pdef);

	// perform setup steps for the planner
	//plan->setup();

	// print the settings for this space
	//_si->printSettings(std::cout);
	//_pdef->print(std::cout);

	// attempt to solve the problem within one second of planning time
  solving_planning = true;
	ob::PlannerStatus solved = plan->solve(SOLVE_PLANNING_TIME_LIMIT);
  solving_planning = false;
	if (solved) {

		std::cout << "Found solution:" << std::endl;
		ob::PathPtr path = _pdef->getSolutionPath();
		og::PathGeometric* pth = _pdef->getSolutionPath()->as<og::PathGeometric>();
		pth->printAsMatrix(std::cout);

		generated_path->header.frame_id = "map";
		geometry_msgs::PoseStamped p;
		p.header.frame_id = "map";
		for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++) {
			const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

			// extract the first component of the state and cast it to what we expect
			const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

			// extract the second component of the state and cast it to what we expect
			const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

			p.pose.position.x = pos->values[0];
			p.pose.position.y = pos->values[1];
			p.pose.position.z = pos->values[2];

			p.pose.orientation.x = rot->x;
			p.pose.orientation.y = rot->y;
			p.pose.orientation.z = rot->z;
			p.pose.orientation.w = rot->w;

			generated_path->poses.push_back( p );
		}
		//ros::Rate r(10);
	}
  plan->clear();
}


/*************************************************************************************************************************************************
Wait until the plan is completed ???
*************************************************************************************************************************************************/
nav_msgs::Path OMPL_PLAN::run(float x_i,float y_i, float z_i, float x_f, float y_f, float z_f) {
  cout<<"\nTrying to plan for:\n\tx_i = "<<x_i<<"  y_i = "<<y_i<<"  z_i = "<<z_i<<" \n\tx_f = "<<x_f<<"  y_f = "<<y_f<<"  z_f = "<<z_f<<endl;
  //if(generated_path)
  // 	delete generated_path;
  generated_path = new nav_msgs::Path();
	ompl_init(x_i, y_i,z_i, x_f, y_f,z_f);
	//boost::thread plan_t( &OMPL_PLAN::plan, this);
	plan();
 // while (ros::ok() && solving_planning)
//	   ros::spinOnce();

  return (*generated_path);
}


/************************************************************************************************************************************************
*************************************************************************************************************************************************
*************************************************************************************************************************************************
*************************************************************************************************************************************************
************************************************************************************************************************************************/



typedef actionlib::SimpleActionClient<px4_planner::FlyAction> Client;

/*************************************************************************************************************************************************
Class that manage the exchange of information with the UAV controller. It uses the class POINTS_INTERATOR and OMPL_PLAN.
quadcopterPosition: quadcopter position
quadcopterFlying: quadcopter is flying or not
*************************************************************************************************************************************************/
class QuadCommanderManager {
  public:
    QuadCommanderManager();
    bool serviceCB(px4_planner::planner_commander_service::Request &req, px4_planner::planner_commander_service::Response &res);
    void doneCB(const actionlib::SimpleClientGoalState& state, const px4_planner::FlyResultConstPtr& result);
    void activeCB();
    void feedbackCB(const px4_planner::FlyFeedbackConstPtr& feedback);
    nav_msgs::Path computePath_OMPLinterface();
    nav_msgs::Path computePath_mod_OMPLinterface();
    void run();

  private:
    ros::NodeHandle _nh;
    POINTS_INTERATOR points_generator;
    OMPL_PLAN op;
    Client ac;
    ros::Publisher _path_pub;
    ros::ServiceServer service_for_master;
    geometry_msgs::Point quadcopterPosition;
    bool quadcopterFlying = false;
    px4_planner::FlyGoal goal;
    int performing_action = 0;
};


QuadCommanderManager::QuadCommanderManager(): ac(ACTIONCLIENTSERVER_NAME, true){
  quadcopterPosition.x = 0; quadcopterPosition.y = 0; quadcopterPosition.z = 0;

  service_for_master = _nh.advertiseService(CLIENTSERVER_NAME, &QuadCommanderManager::serviceCB, this);
  cout<<"Published the service: "<<CLIENTSERVER_NAME<<", to receive the master's commands";

  _path_pub = _nh.advertise<nav_msgs::Path>("path", 0);
  ROS_INFO("Waiting for the action server to start");
  ac.waitForServer();
  ROS_INFO("Action server started.");
}


bool QuadCommanderManager::serviceCB(px4_planner::planner_commander_service::Request &req, px4_planner::planner_commander_service::Response &res) {

  cout<<endl<<"----------------------------------"<<endl<<"---> A new goal is arrived from the MASTER!!"<<endl;
  cout<<"\t  x: "<<req.x<<"  y: "<<req.y<<"  qr_code: "<<req.qr_code<<" land_mask: "<<unsigned(req.land_mask)<<endl<<endl;

/************************************ 1. (possible) Take off action ***********************************/
  if (!quadcopterFlying) {
    goal.path_vel = PATH_VEL_MAX; goal.qr_code = -1;
    goal.command_mask = px4_planner::FlyGoal::TAKEOFF;
    geometry_msgs::PoseStamped p;
    p.pose.position.z = STARTING_HEIGHT;
    goal.path.poses.push_back(p);
    performing_action = goal.command_mask;
    cout<<endl<<"----------------------------------"<<endl<<"---> A new goal is going to be send: Take off"<<endl;
    ac.sendGoal(goal, boost::bind(&QuadCommanderManager::doneCB,this,_1,_2), NULL, boost::bind(&QuadCommanderManager::feedbackCB,this,_1));
    ac.waitForResult();
    cout<<"---> The sent goal has been accomplished"<<endl;
  }

  cout<<endl<<"---------------- Computing checkpoints ----------------";
  points_generator.create_checkPoints(quadcopterPosition.x,quadcopterPosition.y,req.x, req.y, CHECKPOINTS_MAXDISTANCE);
  points_generator.print_checkpoints();

  /************************************ 2. Fallow path action ************************************/
  goal.qr_code = -1;
  goal.command_mask = px4_planner::FlyGoal::FOLLOWPATH;
  performing_action = goal.command_mask;
  while (points_generator.haveMorePoints()) {

    ros::spinOnce();  //update the OMPL octomap
    if (points_generator.index == 0 || points_generator.index == points_generator.points_number-2) {
      goal.path_vel = PATH_VEL_MIN;
    }
    else {
      goal.path_vel = PATH_VEL_MAX;
    }
    cout<<endl<<"----------------------------------"<<endl<<endl<<"----> A new goal is going to be send after the planning: Fallow path, vel: "<<goal.path_vel<<endl;
    goal.path = computePath_mod_OMPLinterface();
    _path_pub.publish(goal.path);
    ac.sendGoal(goal, boost::bind(&QuadCommanderManager::doneCB,this,_1,_2), NULL, boost::bind(&QuadCommanderManager::feedbackCB,this,_1));
    ac.waitForResult();
    cout<<"---> The sent goal has been accomplished"<<endl;
    cout<<"Quadcopter position: x: "<<quadcopterPosition.x<<" y: "<<quadcopterPosition.y<<" z: "<<quadcopterPosition.z<<endl;
  }

  /************************************ 3. End path advertising action ************************************/
  goal.path_vel = PATH_VEL_MAX;   goal.qr_code = -1;
  goal.command_mask = px4_planner::FlyGoal::END_PATH;
  cout<<endl<<"----------------------------------"<<endl<<"---> A new goal is going to be send: End Path"<<endl;
  ac.sendGoal(goal, boost::bind(&QuadCommanderManager::doneCB,this,_1,_2), NULL, boost::bind(&QuadCommanderManager::feedbackCB,this,_1));
  ac.waitForResult();
  cout<<"---> The sent goal has been accomplished"<<endl;


  /************************************ 4. (possibile) Land action ************************************/
  //if (req.land_mask != px4_planner::planner_commander_service::NO_LAND) {
  if (req.land_mask != 0) {
    goal.path_vel = PATH_VEL_MAX;
    goal.command_mask = px4_planner::FlyGoal::QrCODE_LAND;
    //if (req.land_mask == px4_planner::planner_commander_service::QR_CODE_LAND)     goal.qr_code = req.qr_code;
    if (req.land_mask == 2) {
      goal.qr_code = req.qr_code;
      quadcopterFlying = false;
      goal.command_mask = px4_planner::FlyGoal::QrCODE_LAND;
      cout<<endl<<"----------------------------------"<<endl<<"---> A new goal is going to be send: LAND on qr code = "<<req.qr_code<<endl;
    }
    else if(req.land_mask == 1)  {
      goal.qr_code = -1;
      goal.command_mask = px4_planner::FlyGoal::LAND;
      quadcopterFlying = false;
      goal.path.poses[0].pose.position.x = req.x;
      goal.path.poses[0].pose.position.y = req.y;
      cout<<endl<<"----------------------------------"<<endl<<"---> A new goal is going to be send: LAND"<<endl;
    }

    ac.sendGoal(goal, boost::bind(&QuadCommanderManager::doneCB,this,_1,_2), NULL, boost::bind(&QuadCommanderManager::feedbackCB,this,_1));
    ac.waitForResult();
    cout<<"---> The sent goal has been accomplished"<<endl;
  }

  res.completed = true;
  cout<<"The MASTER service has been completely SATISFIED !!  :):):):)"<<endl<<endl<<endl;;

  return true;
}

void QuadCommanderManager::feedbackCB(const px4_planner::FlyFeedbackConstPtr& feedback) {}

void QuadCommanderManager::doneCB(const actionlib::SimpleClientGoalState& state, const px4_planner::FlyResultConstPtr& result) {
	if (performing_action == px4_planner::FlyGoal::LAND || performing_action == px4_planner::FlyGoal::QrCODE_LAND)
		quadcopterFlying = false;
	else if (performing_action == px4_planner::FlyGoal::TAKEOFF)
		quadcopterFlying = true;
}

void QuadCommanderManager::activeCB() {
  cout<<"Goal just went active"<<endl;
}

nav_msgs::Path QuadCommanderManager::computePath_OMPLinterface() {

  static float z_i = STARTING_HEIGHT;
  static float z_f = STARTING_HEIGHT;

  float x_i = points_generator.get_current_x_checkpoint();
  float y_i = points_generator.get_current_y_checkpoint();
  points_generator.index++;
  float x_f = points_generator.get_current_x_checkpoint();
  float y_f = points_generator.get_current_y_checkpoint();

  z_i = z_f;

  bool right_path_computed = false;
  nav_msgs::Path path;
  while (ros::ok() && !right_path_computed) {
  	path = op.run(x_i, y_i, z_i, x_f, y_f, z_f);
    if (path.poses.size() != 0) {
      geometry_msgs::Point point = path.poses[path.poses.size()-1].pose.position;   //just get the last point of the path
      if (x_f != point.x || y_f != point.y) {     //if the planned last point is not the desired final point (x_f y_f)
        right_path_computed = false;
        z_f = z_f + INCREASE_HEIGHT;
        cout<<"Failed to compute path! Taking the next checkpoint:"<<x_f<<"\t"<<y_f<<endl;
      }
      else
        right_path_computed = true;
    }
    else {
      z_f = z_f+INCREASE_HEIGHT;
      std::cout<<"Catched OMPL exception----------------------------------> Failed to compute path! Increasing z to "<<z_f<<std::endl;
    }
  }

  quadcopterPosition = path.poses[path.poses.size()-1].pose.position;
  return path;
}

nav_msgs::Path QuadCommanderManager::computePath_mod_OMPLinterface() {

  static float z_i = STARTING_HEIGHT;
  static float z_f = STARTING_HEIGHT;
  static bool height_increased = false;
  static int wait_before_decrease_altitude = 0;
  int extra_path = 0;

  float x_i = points_generator.get_current_x_checkpoint();
  float y_i = points_generator.get_current_y_checkpoint();
  points_generator.index++;
  float x_f = points_generator.get_current_x_checkpoint();
  float y_f = points_generator.get_current_y_checkpoint();


  z_i = z_f;
  if (!wait_before_decrease_altitude) {
    z_f = STARTING_HEIGHT;
  }

  bool right_path_computed = false;
  nav_msgs::Path path;
  height_increased = false;
  while (ros::ok() && !right_path_computed) {
  	path = op.run(x_i, y_i, z_i, x_f, y_f, z_f);
    if (path.poses.size() != 0) {
      geometry_msgs::Point point = path.poses[path.poses.size()-1].pose.position;   //just get the last point of the path
      if (x_f != point.x || y_f != point.y) {     //if the planned last point is not the desired final point (x_f y_f)
        right_path_computed = false;
        z_f = z_f + INCREASE_HEIGHT;
        cout<<"Failed to compute path! Increasing z to "<<z_f<<endl;
      }
      else {
        right_path_computed = true;
      }
    }
    else {
      z_i = z_i + INCREASE_HEIGHT;
      z_f = z_i;
      extra_path++;
      cout<<"----------------------------------> Failed to compute path! Increasing z initial to "<<z_i<<endl;
    }
  }

  if (z_f > (z_i-extra_path*INCREASE_HEIGHT)) { wait_before_decrease_altitude = NPATH_BEFORE_DECREASE_ALTITUDE; }
  else { wait_before_decrease_altitude--; }

  if (extra_path) {
  	geometry_msgs::PoseStamped temp_pose;
  	temp_pose.pose.position.x = x_i;
	  temp_pose.pose.position.y = y_i;
  	temp_pose.pose.position.z = z_i - extra_path*INCREASE_HEIGHT;
    temp_pose.pose.orientation.x = 0;
    temp_pose.pose.orientation.y = 0;
    temp_pose.pose.orientation.z = 0;
    temp_pose.pose.orientation.w = 1;


    nav_msgs::Path temp_path;
    temp_path.poses.push_back(temp_pose);

    for (int i = 0; i<path.poses.size(); i++) {
      temp_path.poses.push_back(path.poses[i]);
    }
    path = temp_path;
  }

  quadcopterPosition = path.poses[path.poses.size()-1].pose.position;
  return path;
}

void QuadCommanderManager::run() {
  ros::spin();
}


int main(int argc, char** argv ) {

  ros::init(argc, argv, "px4_planner_node");

  QuadCommanderManager manager;
  manager.run();

  return 0;
}

