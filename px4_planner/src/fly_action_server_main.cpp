
#include "px4_planner/fly_action_server.h"
#define ACTIONCLIENTSERVER_NAME "PathCommand_actionserver"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fly_action_server_node");

  FlyAction fly_object(ACTIONCLIENTSERVER_NAME);
	fly_object.run();

	while(ros::ok());
  return 0;
}
