//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "formation_control/formation_control.h"

using namespace Eigen;
using namespace std;
//Constructor
FormationController::FormationController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private):
  nh_(nh),
  nh_private_(nh_private),
  num_vehicles_(3),
  loop_dt_(0.01) {

  cmdloop_timer_ = nh_.createTimer(ros::Duration(loop_dt_), &FormationController::cmdloopCallback, this); // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &FormationController::statusloopCallback, this); // Define timer for constant loop rate

  for(int i = 0; i < num_vehicles_; i++){
    /**
    * @todo Assign arbitrary name spaces
    * @body We need to be able to assign arbitrary name spaces
    */
    vehicle_vector_.emplace_back(nh_, nh_private_, "uav" + std::to_string(i+1));
  }

  formation_pos_ << 0.0, 0.0, 2.0;
  formation_angular_vel_ << 0.0, 0.0, 0.0;
  formation_linear_vel_ << 0.0, 0.0, 0.5;

  vehicle_vector_[0].SetVertexPosition(Eigen::Vector3d(1.0, 0.0, 0.0));
  vehicle_vector_[1].SetVertexPosition(Eigen::Vector3d(0.0, 1.0, 0.0));
  vehicle_vector_[2].SetVertexPosition(Eigen::Vector3d(-1.0, 0.0, 0.0));

}

FormationController::~FormationController() {
  //Destructor
}

void FormationController::cmdloopCallback(const ros::TimerEvent& event){

    /**
    * @todo Get formation reference from a proper interface
    * @body Get rid of the formation states being updated automatically
    */

  Eigen::Matrix4d Qx;
  Eigen::Vector4d d_formation_att;
  Eigen::Vector3d omega = formation_angular_vel_;

  Qx <<      0.0, -omega(0), -omega(1), -omega(2),
        omega(0),       0.0,  omega(2), -omega(1),
        omega(1), -omega(2),       0.0,  omega(0),
        omega(2),  omega(1), -omega(0),       0.0;

  formation_pos_ = formation_pos_ + formation_linear_vel_ * loop_dt_;
  d_formation_att = Qx * formation_att_;
  formation_att_ = formation_att_ + d_formation_att * loop_dt_;
  
  UpdateVrbVertexStates();

}

void FormationController::statusloopCallback(const ros::TimerEvent& event){

}

void FormationController::UpdateVrbVertexStates(){
  //Update vertex states on virtual rigid body
  for(int i = 0; i < num_vehicles_; i++){
    Eigen::Vector3d vehicle_pos;
    Eigen::Vector3d vehicle_vel;
    Eigen::Vector3d vertex_position;

    vertex_position = vehicle_vector_[i].GetVertexPosition();
    CalculateVertexStates(vertex_position, vehicle_pos, vehicle_vel);
    vehicle_vector_[i].SetReferenceState(vehicle_pos, vehicle_vel);
  }
}

void FormationController::CalculateVertexStates(Eigen::Vector3d vrb_position, Eigen::Vector3d &pos, Eigen::Vector3d &vel){
  Eigen::Matrix3d formation_rotmat;

  formation_rotmat = quat2RotMatrix(formation_att_);
  pos = formation_pos_ + formation_rotmat * vrb_position;
  vel = formation_linear_vel_ + formation_angular_vel_.cross(vrb_position);

}

Eigen::Matrix3d FormationController::quat2RotMatrix(Eigen::Vector4d q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d FormationController::rot2Quaternion(Eigen::Matrix3d R){
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}