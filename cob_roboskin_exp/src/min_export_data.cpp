#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN2.h"


using namespace std;

	
class SaveMin
{
	public:
	 ros::NodeHandle n;
	 void chatterCallback(const cob_roboskin::roboskinCAN2 msg);
	 
};
	
void SaveMin::chatterCallback(const cob_roboskin::roboskinCAN2 msg)
{

  cout << ros::Time::now() << " ";

  int min1;
  int min2;
  int min3;
  int min4;

	if (n.getParam("/min1", min1)){
		}
	if (n.getParam("/min2", min2)){
		}
	if (n.getParam("/min3", min3)){
		}
	if (n.getParam("/min4", min4)){
		}

  cout << min1 << " ";
  cout << min2 << " ";
  cout << min3 << " ";
  cout << min4 << " ";

  cout << "\n";
}

int main(int argc, char **argv)
{
  //ros::init(argc, argv, "listener");
  ros::init(argc, argv, "SaveMin");
  SaveMin my_cvi = SaveMin();
  //ros::NodeHandle n;
 
  ros::Subscriber sub = my_cvi.n.subscribe("roboskin", 1000, &SaveMin::chatterCallback, &my_cvi);
  ros::spin();

  return 0;
}

