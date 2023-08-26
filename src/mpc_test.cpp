#include "mpcc/mpc_test.h"
#include <signal.h>

void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

int main(int argc, char **argv){

  signal(SIGINT, signal_handler); // to exit program when ctrl+c

  ros::init(argc, argv, "mpc_node");
  ros::NodeHandle n("~");
  LSH_MPC LSH_MPC(n);    
  ros::AsyncSpinner spinner(5); // Use 5 threads -> 5 callbacks
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
