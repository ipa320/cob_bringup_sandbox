#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN.h"

using namespace std;

void chatterCallback(const cob_roboskin::roboskinCAN msg)
{
  cout << "ID: " << msg.id << "\t";
  for (int i = 0; i < 12; i++)
  {
	cout << msg.data[i] << " ";

  }
  cout << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("roboskin", 10000, chatterCallback);
  ros::spin();

  return 0;
}
