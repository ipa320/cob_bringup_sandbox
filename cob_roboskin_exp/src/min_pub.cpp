#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN2.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "actionlib_msgs/GoalID.h"



using namespace std;

class CalcMin
{
	public:
	  ros::NodeHandle n;
	  ros::Publisher pub;
	  //ros::Subscriber sub;
          std_msgs::Int32MultiArray req;
	  void chatterCallback(const cob_roboskin::roboskinCAN2 msg);
		

};
	
void CalcMin::chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{

int min1;
int min2;
int min3;
int min4;

  req.data = std::vector<int>(8);
  req.data[0] = msg.data1;
  req.data[1] = msg.data2;
  req.data[2] = msg.data3;
  req.data[3] = msg.data4;

	if (n.getParam("/min1", min1)){
		  req.data[4] = min1;
		}
	if (n.getParam("/min2", min2)){
		  req.data[5] = min2;
		}
	if (n.getParam("/min3", min3)){
		  req.data[6] = min3;
		}
	if (n.getParam("/min4", min4)){
		  req.data[7] = min4;
		}

  pub = n.advertise<std_msgs::Int32MultiArray>("/min_pub",1000); 
  pub.publish(req);

}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "CalcMin");
  CalcMin my_cvi = CalcMin();
  ros::Subscriber sub = my_cvi.n.subscribe("roboskin", 1000, &CalcMin::chatterCallback, &my_cvi);
  ros::spin();

  return 0;
}

