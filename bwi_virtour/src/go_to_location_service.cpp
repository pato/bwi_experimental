#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "bwi_virtour/GoToLocation.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

bool goToLocation(bwi_virtour::GoToLocation::Request &req,
    bwi_virtour::GoToLocation::Response &res) {
  ROS_INFO("requesting goToLocation: %s", req.location.c_str());
  res.result = 1;
  ROS_INFO("sending back response: [%ld]", (long int)res.result);
  return true;
}
    

int main(int argc, char**argv) {
  ros::init(argc, argv, "go_to_location_service_node");
  ros::NodeHandle n;
  
  string location = "l3_414b";

  ros::ServiceServer service = n.advertiseService("go_to_location", goToLocation);
  ROS_INFO("GoToLocation Service Started");
  ros::spin();
  ROS_INFO("Done spinning");
  
  ROS_INFO_STREAM("going to " << location);

  return 0;
  
  Client client("/action_executor/execute_plan", true);
  client.waitForServer();
  
  bwi_kr_execution::ExecutePlanGoal goal;
  
  bwi_kr_execution::AspRule rule;
  bwi_kr_execution::AspFluent fluent;
  fluent.name = "not at";
  
  fluent.variables.push_back(location);
 
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  
  ROS_INFO("sending goal");
  client.sendGoalAndWait(goal);
  
  if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
  } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
  } else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
  } else {
     ROS_INFO("Terminated");
  }
    
  return 0;
}
