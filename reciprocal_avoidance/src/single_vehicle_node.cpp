//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "reciprocal_avoidance/single_vehicle.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"reciprocal_avoidance");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  SingleVehicle singlevehicleController(nh, nh_private);
  ros::spin();
  return 0;
}
