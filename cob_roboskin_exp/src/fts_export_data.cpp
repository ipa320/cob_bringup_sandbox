#include "ros/ros.h"
#include "fts_Omega160/fts.h"

using namespace std;

void chatterCallback(const fts_Omega160::fts msg)
{
  cout << ros::Time::now() << " ";
  cout << msg.Fx << " ";
  cout << msg.Fy << " ";
  cout << msg.Fz << " ";
  cout << msg.Mx << " ";
  cout << msg.My << " ";
  cout << msg.Mz << " ";

  cout << "\n";

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener2");
  ros::NodeHandle n;
  ros::Subscriber sub2 = n.subscribe("FTSData", 1000, chatterCallback);
  ros::spin();

  return 0;
}

