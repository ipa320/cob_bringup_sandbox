#include "ros/ros.h"
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"


using namespace std;

void chatterCallback(const pr2_controllers_msgs::JointTrajectoryControllerState msg)
{

  cout << ros::Time::now() << " ";
  for (int i=0;i<7;i++){
  	cout  << msg.actual.velocities[i] << " ";
}
  cout << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener8");
  ros::NodeHandle n1;
  ros::Subscriber sub1 = n1.subscribe("arm_controller/state", 10000, chatterCallback);
  ros::spin();

  return 0;
}

