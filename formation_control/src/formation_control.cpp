//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "formation_control/formation_control.h"

using namespace Eigen;
using namespace std;
//Constructor
FormationController::FormationController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  num_vehicles_(3) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.03), &FormationController::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &FormationController::statusloopCallback, this); // Define timer for constant loop rate

  for(int i = 0; i < num_vehicles_; i++){
    vehicle_vector_.emplace_back(nh_, nh_private_, "uav" + std::string(i+1));
  }

  formation_center_ << 0.0, 0.0, 2.0;
}

FormationController::~FormationController() {
  //Destructor
}

void FormationController::cmdloopCallback(const ros::TimerEvent& event){

}

void FormationController::statusloopCallback(const ros::TimerEvent& event){

}