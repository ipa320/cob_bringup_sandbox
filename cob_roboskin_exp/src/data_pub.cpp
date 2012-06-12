#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN2.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"
#include "actionlib_msgs/GoalID.h"
#include "fts_Omega160/fts.h"


using namespace std;

class CalcMin
{
	public:
	  ros::NodeHandle n;
	  ros::Publisher pub;
	  //ros::Subscriber sub;
          std_msgs::Int32MultiArray req;
	  void chatterCallback1(const fts_Omega160::fts msg);
	  void chatterCallback(const cob_roboskin::roboskinCAN2 msg);
		

};
	
void CalcMin::chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{
  n.setParam("/data1", msg.data1);
  n.setParam("/data2", msg.data2);
  n.setParam("/data3", msg.data3);
  n.setParam("/data4", msg.data4);
}

void CalcMin::chatterCallback1(const fts_Omega160::fts msg)
{
int data1;
int data2;
int data3;
int data4;
int stop;

  n.setParam("/stop", 0);
  req.data = std::vector<int>(8);
  req.data[4] = msg.Fx;
  req.data[5] = msg.Fy;
  req.data[6] = msg.Fz;

	if (n.getParam("/data1", data1)){
		  req.data[0] = data1;
		}
	if (n.getParam("/data2", data2)){
		  req.data[1] = data2;
		}
	if (n.getParam("/data3", data3)){
		  req.data[2] = data3;
		}
	if (n.getParam("/data4", data4)){
		  req.data[3] = data4;
		}
	if (n.getParam("/stop", stop)){
		  req.data[7] = stop;
		}


  pub = n.advertise<std_msgs::Int32MultiArray>("/min_pub",1000); 
  pub.publish(req);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "CalcMin");
  CalcMin my_cvi = CalcMin();
  ros::Subscriber sub1 = my_cvi.n.subscribe("FTSData", 1000, &CalcMin::chatterCallback1, &my_cvi);
  ros::Subscriber sub = my_cvi.n.subscribe("roboskin", 1000, &CalcMin::chatterCallback, &my_cvi);
  ros::spin();

  return 0;
}

