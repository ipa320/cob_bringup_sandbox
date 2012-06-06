#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN2.h"


using namespace std;

void chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{

  int data1 = msg.data1;
  int data2 = msg.data2;
  int data3 = msg.data3;
  int data4 = msg.data4;

  cout << ros::Time::now() << " ";
  cout << data1 << " ";
  cout << data2 << " ";
  cout << data3 << " ";
  cout << data4 << " ";


		
  cout << "\n";
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener1");
  ros::NodeHandle n3;
  ros::Subscriber sub3 = n3.subscribe("roboskin", 1000,chatterCallback);

  ros::spin();

  return 0;
}

