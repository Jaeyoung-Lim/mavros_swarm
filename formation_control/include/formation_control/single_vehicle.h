//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef SINGLE_VEHICLE_H
#define SINGLE_VEHICLE_H

#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"


class SingleVehicle
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    ros::Publisher setpoint_publisher_;

    ros::Timer cmdloop_timer_;
    ros::Timer statusloop_timer_;
    
    ros::Time last_request_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    bool sim_enable_;

    
    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    void PublishSetpoint();

  public:
    SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~SingleVehicle();    
    
};


#endif