#include "tourManager.h"
#include "bwi_virtour/RequestTour.h"
#include "bwi_virtour/PingTour.h"
#include "bwi_virtour/GetTourState.h"

TourManager* tm;

bool requestTour(bwi_virtour::RequestTour::Request &req,
    bwi_virtour::RequestTour::Response &res) {

  if (tm->tourAllowed) {
    if (!tm->tourInProgress) {
      /* allow tour */
      //TODO add mutex
      tm->tourLeader = req.user;
      tm->tourInProgress = true;
      tm->tourStartTime = ros::Time::now();
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

  if (req.user.compare(tm->tourLeader) == 0) {
    tm-> lastPingTime = ros::Time::now();
    res.result = 1;
  } else {
    res.result = TourManager::ERROR_NOTTOURLEADER;
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
  res.tourLeader = tm->tourLeader;

  return true;
}

int main(int argc, char **argv){
  tm = new TourManager(true);

  ros::init(argc, argv, "tourManager");
  ros::NodeHandle n;

  /* Advertise services */
  ros::ServiceServer request_service = n.advertiseService("request_tour", requestTour);
  ros::ServiceServer ping_service = n.advertiseService("ping_tour", pingTour);

  ros::spin();
  return 0;
}
