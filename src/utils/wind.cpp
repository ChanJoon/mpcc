#include <signal.h>

#include "utils/wind_disturbance.h"
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

int main(int argc, char **argv) {
  LOG_INIT(argv[0]);
  ros::init(argc, argv, "wind_node");
  ros::NodeHandle n("~");

  wind_disturbance wdt_(n);

  signal(SIGINT, signal_handler);  // to exit program when ctrl+c

  ros::AsyncSpinner spinner(2);  // Use 2 threads
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
