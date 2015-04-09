#include "tourManager.h"
#include "bwi_virtour/RequestTour.h"
#include "bwi_virtour/PingTour.h"
#include "bwi_virtour/GetTourState.h"
#include "bwi_virtour/LeaveTour.h"
#include "bwi_virtour/IsLeader.h"

TourManager* tm;

bool requestTour(bwi_virtour::RequestTour::Request &req,
    bwi_virtour::RequestTour::Response &res) {

  if (tm->tourAllowed) {
    if (!tm->tourInProgress) {
      //TODO add mutex
      tm->tourLeader = req.user;
      tm->tourInProgress = true;
      tm->tourStartTime = ros::Time::now();
      res.startTime = tm->tourStartTime.toSec();
      res.result = tm->tourDuration;

    } else {
      res.result = TourManager::ERROR_TOURINPROGRESS;
    }
  } else {
    res.result = TourManager::ERROR_NOTOURALLOWED;
  }

  return true;
}

bool pingTour(bwi_virtour::PingTour::Request &req,
    bwi_virtour::PingTour::Response &res) {

  if (tm->tourAllowed) {
    if (tm->tourInProgress) {
      if (req.user.compare(tm->tourLeader) == 0) {
        tm-> lastPingTime = ros::Time::now();
        res.result = 1;
      } else {
        res.result = TourManager::ERROR_NOTTOURLEADER;
      }
    } else {
      res.result = TourManager::ERROR_NOTOURINPROGRESS;
    }
  } else {
    res.result = TourManager::ERROR_NOTOURALLOWED;
  }
  return true;
}

bool getTourState(bwi_virtour::GetTourState::Request &req,
    bwi_virtour::GetTourState::Response &res) {

  res.tourAllowed = tm->tourAllowed;
  res.tourInProgress = tm->tourInProgress;
  res.tourDuration = tm->tourDuration;
  res.tourStartTime = tm->tourStartTime.toSec();
  res.lastPingTime = tm->lastPingTime.toSec();
  res.tourLeader = ""; //tm->tourLeader; don't actually want to leak this

  return true;
}

bool leaveTour(bwi_virtour::LeaveTour::Request &req,
    bwi_virtour::LeaveTour::Response &res) {

  if (tm->tourInProgress && req.user.compare(tm->tourLeader) == 0) {
    //TODO add mutex
    tm->tourInProgress = false;
    tm->tourLeader = "";

    res.result = 1;
  } else {
    res.result = TourManager::ERROR_NOTTOURLEADER;
  }

  return true;
}

bool isLeader(bwi_virtour::IsLeader::Request &req,
    bwi_virtour::IsLeader::Response &res) {
 
  if (tm->tourInProgress) {
    if (req.user.compare(tm->tourLeader) == 0) {
      res.result = 1;
    } else {
      res.result = TourManager::ERROR_NOTTOURLEADER;
    }
  } else {
    res.result = TourManager::ERROR_NOTOURALLOWED;
  }

  return true;
}

int main(int argc, char **argv){
  tm = new TourManager(true);

  ros::init(argc, argv, "tourManager");
  ros::NodeHandle n;

  /* Advertise services */
  ros::ServiceServer request_service = n.advertiseService("tourManager/request_tour", requestTour);
  ros::ServiceServer ping_service = n.advertiseService("tourManager/ping_tour", pingTour);
  ros::ServiceServer get_tour_state_service = n.advertiseService("tourManager/get_tour_state", getTourState);
  ros::ServiceServer leave_service = n.advertiseService("tourManager/leave_tour", leaveTour);

  ros::spin();
  return 0;
}
