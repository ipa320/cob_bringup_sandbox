#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN.h"
#include "cob_roboskin/roboskinCAN2.h"
#include "actionlib_msgs/GoalID.h"


using namespace std;

class StopPub
{
	public:
	  ros::NodeHandle n;
	  ros::Publisher pub;
	  void chatterCallback(const cob_roboskin::roboskinCAN2 msg);
	  

};
	
void StopPub::chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{
  int min1;
  int min2;
  int min3;
  int min4;

  if (n.getParam("/min1", min1)){
	if (msg.data1 < min1-5){
			actionlib_msgs::GoalID req;
			pub = n.advertise<actionlib_msgs::GoalID>("/arm_controller/joint_trajectory_action/cancel",1000); 
			pub.publish(req);
			cout << "id1" << " ";
		}
}
  if (n.getParam("/min2", min2)){
	if (msg.data2 < min2-5){
		actionlib_msgs::GoalID req;
		pub = n.advertise<actionlib_msgs::GoalID>("/arm_controller/joint_trajectory_action/cancel",1000); 
		pub.publish(req);
		cout << "id2" << " ";
	}
}
  if (n.getParam("/min3", min3)){
	if (msg.data3 < min3-5){
		actionlib_msgs::GoalID req;
		pub = n.advertise<actionlib_msgs::GoalID>("/arm_controller/joint_trajectory_action/cancel",1000); 
		pub.publish(req);
		cout << "id3" << " ";
}
  if (n.getParam("/min4", min4)){
}
	if (msg.data4 < min4-5){
		actionlib_msgs::GoalID req;
		pub = n.advertise<actionlib_msgs::GoalID>("/arm_controller/joint_trajectory_action/cancel",1000); 
		pub.publish(req);
		cout << "id4" << " ";
	}
}

	cout << "\n";
}


int main(int argc, char **argv)
{
  //ros::init(argc, argv, "listener");
  ros::init(argc, argv, "StopPub");
  StopPub my_cvi = StopPub();
  //ros::NodeHandle n;
 
  ros::Subscriber sub = my_cvi.n.subscribe("roboskin", 1000, &StopPub::chatterCallback, &my_cvi);
  ros::spin();

  return 0;
}

