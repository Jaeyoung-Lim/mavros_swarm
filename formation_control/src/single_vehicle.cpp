//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "formation_control/single_vehicle.h"


//Constructor
SingleVehicle::SingleVehicle(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private){

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.03), &SingleVehicle::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &SingleVehicle::statusloopCallback, this); // Define timer for constant loop rate
 
  setpoint_publisher_ = nh_.advertise<mavros_msgs::PositionTarget>("/uav1/mavros/setpoint_raw/local", 1);
  
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

SingleVehicle::~SingleVehicle() {
  //Destructor
}

void SingleVehicle::cmdloopCallback(const ros::TimerEvent& event){
  PublishSetpoint();

}

void SingleVehicle::statusloopCallback(const ros::TimerEvent& event){
    if(sim_enable_){
    // Enable OFFBoard mode and arm automatically
    // This is only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if( !current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
        if( arming_client_.call(arm_cmd_) && arm_cmd_.response.success){
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
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