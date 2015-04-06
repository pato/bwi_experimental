#include<string>
#include <ros/ros.h>

class TourManager{
  public:
    bool tourAllowed;
    bool tourInProgress;
    ros::Time tourStartTime;
    long tourDuration; /* in seconds */
    std::string tourLeader;

    static const int ERROR_NOTOURALLOWED = -2;
    static const int ERROR_TOURINPROGRESS = -3;

    TourManager(bool tourAllowed) : tourAllowed(tourAllowed),
      tourInProgress(false), tourStartTime(0) {

      tourDuration = 30*60;
    }
};
