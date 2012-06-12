#include "ros/ros.h"
#include "fts_Omega160/fts.h"

using namespace std;

void chatterCallback(const fts_Omega160::fts msg)
{
  cout << "Fx: " << msg.Fx << "\t";
  cout << "Fy: " << msg.Fy << "\t";
  cout << "Fz: " << msg.Fz << "\t";
  cout << "Mx: " << msg.Mx << "\t";
  cout << "My: " << msg.My << "\t";
  cout << "Mz: " << msg.Mz << "\t";

  
  cout << "\n";
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("FTSData", 1000, chatterCallback);
  ros::spin();

  return 0;
}

