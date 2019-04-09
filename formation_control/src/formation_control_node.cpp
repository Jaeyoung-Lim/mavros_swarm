//  July/2018, ETHZ, Jaeyoung Lim, jalim@student.ethz.ch

#include "formation_control/formation_control.h"

int main(int argc, char** argv) {
  ros::init(argc,argv,"formation_control");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  FormationController geometricController(nh, nh_private);
  ros::spin();
  return 0;
}
