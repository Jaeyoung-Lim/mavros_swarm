//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "formation_control/single_vehicle.h"


//Constructor
SingleVehicle::SingleVehicle(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private,
                             std::string name):
  nh_(nh),
  nh_private_(nh_private),
  vehicle_name_(name) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.03), &SingleVehicle::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &SingleVehicle::statusloopCallback, this); // Define timer for constant loop rate

  setpoint_publisher_ = nh_.advertise<mavros_msgs::PositionTarget>("/"+vehicle_name_+"/mavros/setpoint_raw/local", 1);

  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/"+vehicle_name_+"/mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/"+vehicle_name_+"/mavros/set_mode");
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
    /**
    * @todo Dangerous behaviros in  automatic disarming / rearming
    * @body This will rearm the vehicle in case of a failure
    */
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if( current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))){
      if( set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent){
        ROS_INFO("Setting to Offboard");
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

void SingleVehicle::SetReferenceState(Eigen::Vector3d ref_position, Eigen::Vector3d ref_velocity){
  reference_pos_ = ref_position;
  reference_vel_ = ref_velocity;

}

void SingleVehicle::PublishSetpoint(){
  /**
  * @todo Fix segfault
  * @body Currently this results in a segfault
  */

  mavros_msgs::PositionTarget msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.type_mask = 64;
  msg.position.x = reference_pos_(0);
  msg.position.y = reference_pos_(1);
  msg.position.z = reference_pos_(2);
  msg.velocity.x = reference_vel_(0);
  msg.velocity.y = reference_vel_(1);
  msg.velocity.z = reference_vel_(2);

  // setpoint_publisher_.publish(msg);
}

void SingleVehicle::SetNameSpace(std::string vehicle_name){
  vehicle_name_ = vehicle_name;

}

void SingleVehicle::SetVertexPosition(Eigen::Vector3d position){
    vrb_vertexpos_ = position;

}

Eigen::Vector3d SingleVehicle::GetVertexPosition(){ return vrb_vertexpos_; }