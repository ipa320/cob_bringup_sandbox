#include "ros/ros.h"
#include "actionlib_msgs/GoalID.h"


using namespace std;

void chatterCallback(const actionlib_msgs::GoalID msg)
{

  cout << ros::Time::now() << " ";
  cout  << "1" << " ";

  cout << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener3");
  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("arm_controller/joint_trajectory_action/cancel", 10000,chatterCallback);
  ros::spin();

  return 0;
}

