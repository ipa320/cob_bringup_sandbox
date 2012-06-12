#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN2.h"
#include "actionlib_msgs/GoalID.h"


using namespace std;

class CalcMin
{
	public:
	  ros::NodeHandle n;
	  void chatterCallback(const cob_roboskin::roboskinCAN2 msg);

};
	
void CalcMin::chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{

  int min1;
  int min2;
  int min3;
  int min4;

	if (n.getParam("/min1", min1)){
		if( min1 == 0 ){
			n.setParam("/min1", 240);
		}
	        else{
			if(msg.data1 < min1 ){
				n.setParam("/min1", msg.data1);
				cout << "new min for id1 " << min1 << "\t";
	 			}
		}
}
	if (n.getParam("/min2", min2))
	 {
		if( min2 == 0 ){
			n.setParam("/min2", 240);
		}
	        else{
			if(msg.data2 < min2 ){
				n.setParam("/min2", msg.data2);
				cout << "new min for id2 " << min2 << "\t";
	 			}
		}

}
	if (n.getParam("/min3", min3))
	 {
		if( min3 == 0 ){
			n.setParam("/min3", 240);
		}
	        else{
			if(msg.data3 < min3 ){
				n.setParam("/min3", msg.data3);
				cout << "new min for id3 " << min3 << "\t";
	 			}
		}

}

	if (n.getParam("/min4", min4))
	 {
		if( min4 == 0 ){
			n.setParam("/min4", 240);
		}
	        else{
			if(msg.data4 < min4 ){
				n.setParam("/min4", msg.data4);
				cout << "new min for id4 " << min4 << "\t";
	 			}
		}

}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "calibration");
  CalcMin my_cvi = CalcMin();
  ros::Subscriber sub = my_cvi.n.subscribe("roboskin", 1000, &CalcMin::chatterCallback, &my_cvi);
  ros::spin();

  return 0;
}

