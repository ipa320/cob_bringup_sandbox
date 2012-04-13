#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cob_hwmonitor/hw_msg.h"

using namespace std;

void chatterCallback(const cob_hwmonitor::hw_msg msg)
{
  int channel = 1;//msg.data[0];
  int specifier = 2;//msg.data[1];
  int status = 3;//msg.data[2];
  int data = 4;//msg.data[3];
  data = data*256 + 5;//msg.data[4];
  cout << "\n";
  if (channel == 0)
  {
    cout << "\n\t\t\t\t\t\t";
    switch(specifier)
    {
      case 0:
	cout << "Current Temperature\n";
	break;
      case 1:
	cout << "Minimum Temperature\n";
	break;
      case 2:
	cout << "Maximum Temperature\n";
	break;
      case 3:
	cout << "Current Voltage\n";
	break;
      case 4:
	cout << "Minimum Voltage\n";
	break;
      case 5:
	cout << "Maximum Voltage\n";
	break;
      case 6:
	cout << "Current Current\n";
	break;
      case 7:
	cout << "Minimum Current\n";
	break;
      case 8:
	cout << "Maximum Current\n";
	break;
      default:
	cout << "ERROR\n";
    }
    cout << "\n";
  }
  cout << "Channel\t\t=\t" << channel << "\n"; 
  cout << "Status\t\t=\t";
  switch(status)
  {
    case 0:
      cout << "No Error\n";
      break;
    case 1:
      cout << "Settings changed\n";
      break;
    case 2:
      cout << "Reset successfull\n";
      break;
    case 3:
      cout << "Invalid channel\n";
      break;
    case 4:
      cout << "Invalid specifier\n";
      break;
    case 5:
      cout << "Invalid sensor\n";
      break;
    case 6:
      cout << "Switch successfull\n";
      break;
    case 7:
      cout << "Invalid data\n";
      break;
    case 8:
      cout << "Out of range\n";
      break;
    default:
      cout << "ERROR\n";
  }
  cout << "Value\t\t=\t" << data << "\n";
}

int main(int argc, char **argv)

{
  ros::init(argc, argv, "hwcpp");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("hwmonitor", 1000, chatterCallback);
  ros::spin();
  
  return 0;
}

