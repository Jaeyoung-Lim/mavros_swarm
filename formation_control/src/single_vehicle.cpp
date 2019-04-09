//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "formation_control/single_vehicle.h"


//Constructor
SingleVehicle::SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private){

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.03), &SingleVehicle::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &SingleVehicle::statusloopCallback, this); // Define timer for constant loop rate
 
  setpoint_publisher_ = nh_.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 1);
}

SingleVehicle::~SingleVehicle() {
  //Destructor
}

void SingleVehicle::cmdloopCallback(const ros::TimerEvent& event){
  PublishSetpoint();

}

void SingleVehicle::statusloopCallback(const ros::TimerEvent& event){

}

void SingleVehicle::PublishSetpoint(){
  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = 64;
  msg.position.x = 0.0;
  msg.position.y = 0.0;
  msg.position.z = 0.0;
  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;

  setpoint_publisher_.publish(msg);

}