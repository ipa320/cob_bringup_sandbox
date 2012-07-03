#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN.h"
#include "cob_roboskin/roboskinCAN2.h"

using namespace std;

void chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{
  /*int data = 0;
  for (int i = 0; i < 12; i++)
  {
  	data = data + msg.data[i];
  }
  data = data / 12;
  
  if (data < 229)
  {
  	cout << "ID: " << msg.id << "\t" << data;
  	cout << "\n";
  }*/
  int data1 = msg.data1;
  int data2 = msg.data2;
  int data3 = msg.data3;
  int data4 = msg.data4;
 
  /*if (data1 < 255)
  	cout << "ID: " << msg.id1 << "\t" << data1 << "\n";
  if (data2 < 255)
	  cout << "ID: " << msg.id2 << "\t" << data2 << "\n";
  if (data3 < 255)
  	cout << "ID: " << msg.id3 << "\t" << data3 << "\n";
  if (data4 < 255)
  	cout << "ID: " << msg.id4 << "\t" << data4 << "\n";*/
	
  cout << ros::Time::now() << " ";
  cout << data1 << " ";
  cout << data2 << " ";
  cout << data3 << " ";
  cout << data4 << " ";
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
