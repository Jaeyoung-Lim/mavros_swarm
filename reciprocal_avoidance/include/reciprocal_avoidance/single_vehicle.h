//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef SINGLE_VEHICLE_H
#define SINGLE_VEHICLE_H

#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "Eigen/Dense"

enum SETPOINT_TYPE
{
    LOCAL_SETPOINT,
    GLOBAL_SETPOINT
};

class SingleVehicle
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    ros::Publisher localsetpoint_publisher_;
    ros::Publisher globalsetpoint_publisher_;
    ros::Subscriber mavstateSub_;

    ros::Timer cmdloop_timer_;
    ros::Timer statusloop_timer_;

    ros::Time last_request_;

    SETPOINT_TYPE setpoint_type_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    Eigen::Vector3d reference_pos_; // Vehicle position in world coordinate
    Eigen::Vector3d reference_vel_; // Vehicle velocity in world coordinate
    Eigen::Vector3d vrb_vertexpos_; // Vehicle position in virtual rigid body coordinate

    bool sim_enable_;
    double loop_dt_;
    std::string vehicle_name_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void PublishGlobalSetpoint();
    void PublishLocalSetpoint();

  public:
    SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle&);
    SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, std::string name);
    virtual ~SingleVehicle();
    void SetReferenceState(Eigen::Vector3d ref_position, Eigen::Vector3d ref_velocity);
    
};
#endif