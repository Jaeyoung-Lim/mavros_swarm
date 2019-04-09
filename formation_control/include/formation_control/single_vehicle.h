//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef SINGLE_VEHICLE_H
#define SINGLE_VEHICLE_H

#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"


class SingleVehicle
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Timer cmdloop_timer_;
    ros::Timer statusloop_timer_;

    ros::Publisher setpoint_publisher_;
    
    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    void PublishSetpoint();

  public:
    SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~SingleVehicle();    
    
};


#endif
