//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <string>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <std_srvs/SetBool.h>


#include "formation_control/single_vehicle.h"

using namespace std;
using namespace Eigen;
class FormationController
{
  private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::vector<std::shared_ptr<SingleVehicle>> vehicle_vector_;
    // SingleVehicle vehicle_;

    ros::Timer cmdloop_timer_;
    ros::Timer statusloop_timer_;

    Eigen::Vector3d formation_pos_;
    Eigen::Vector4d formation_att_;
    Eigen::Vector3d formation_linear_vel_;
    Eigen::Vector3d formation_angular_vel_;
    
    int num_vehicles_;
    double loop_dt_;

    void cmdloopCallback(const ros::TimerEvent& event);
    void statusloopCallback(const ros::TimerEvent& event);
    void UpdateVrbVertexStates();
    void CalculateVertexStates(Eigen::Vector3d vrb_position, Eigen::Vector3d &pos, Eigen::Vector3d &vel);
    Eigen::Vector4d rot2Quaternion(Eigen::Matrix3d R);
    Eigen::Matrix3d quat2RotMatrix(Eigen::Vector4d q);


  public:
    FormationController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~FormationController();
};


#endif
