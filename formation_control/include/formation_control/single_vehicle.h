//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef SINGLE_VEHICLE_H
#define SINGLE_VEHICLE_H

#include <ros/ros.h>
#include "mavros_msgs/PositionTarget.h"
#include "mavros_msgs/GlobalPositionTarget.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/HomePosition.h"
#include "Eigen/Dense"
#include "geodetic_utils/geodetic_conv.hpp"


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
    ros::Subscriber homeposSub_;

    ros::Timer cmdloop_timer_;
    ros::Timer statusloop_timer_;

    ros::Time last_request_;

    mavros_msgs::State current_state_;
    mavros_msgs::SetMode offb_set_mode_;
    mavros_msgs::CommandBool arm_cmd_;

    geodetic_converter::GeodeticConverter geod_converter_global_origin_;
    geodetic_converter::GeodeticConverter geod_converter_home_;

    Eigen::Vector3d reference_pos_; // Vehicle position in world coordinate
    Eigen::Vector3d reference_vel_; // Vehicle velocity in world coordinate
    Eigen::Vector3d local_reference_pos_; // Vehicle position in world coordinate
    Eigen::Vector3d local_reference_vel_; // Vehicle velocity in world coordinate
    Eigen::Vector3d vrb_vertexpos_; // Vehicle position in virtual rigid body coordinate
    Eigen::Vector3d initial_pos_; //Initial vehicle poistion with respect to the world frame
    Eigen::Vector3d global_origin_, global2local_origin_;
    Eigen::Vector3d global_pos_, global_pos_utm_;
    bool is_auto_arming_;
    double loop_dt_;
    
    std::string vehicle_name_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    void mavstateCallback(const mavros_msgs::State::ConstPtr& msg);
    void HomePosCallback(const mavros_msgs::HomePosition::ConstPtr& msg);
    void PublishLocalSetpoint();

  public:
    SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private, std::string name);
    virtual ~SingleVehicle();
    void SetReferenceState(Eigen::Vector3d ref_position, Eigen::Vector3d ref_velocity);
    void SetNameSpace(std::string vehicle_name);
    void SetVertexPosition(Eigen::Vector3d position);
    void SetInitialPosition(Eigen::Vector3d position);
    void SetGlobalOrigin(Eigen::Vector3d origin);
    Eigen::Vector3d GetVertexPosition();
    
};
#endif