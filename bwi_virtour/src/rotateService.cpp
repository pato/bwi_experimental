#include "bwi_kr_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include "bwi_virtour/GoToLocation.h"

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

Client* client;

using namespace std;

bool rotate(bwi_virtour::Rotate::Request &req,
    bwi_virtour::Rotate::Response &res) {
  
  ROS_INFO("Received rotate request");
  float rotateDelta = req.rotateDelta;

  res.result = 1;
  return true;
}

int main(int argc, char**argv) {
  ros::init(argc, argv, "rotate_service_node");
  ros::NodeHandle n;

  //client = new Client("/action_executor/execute_plan", true);

  ros::ServiceServer service = n.advertiseService("rotate", rotate);
  ROS_INFO("Rotate Service Started");

  ros::spin();
  ROS_INFO("Done spinning");
  return 0;
}
