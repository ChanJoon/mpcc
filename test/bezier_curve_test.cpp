#include <mpcc/bezier_curve_test.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "bezier_curve_node");
  ros::NodeHandle nh;
  BEZIER_TEST bezier(nh);

  ros::spin();

  return 0;
}
