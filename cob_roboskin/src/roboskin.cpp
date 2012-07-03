#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN.h"
#include "libpcan/libpcan.h"
#include "fcntl.h"

#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roboskin");
  ros::NodeHandle n;
  ros::Publisher roboskin_pub = n.advertise<cob_roboskin::roboskinCAN>("roboskin", 1);
  
  HANDLE h;
  int baud;
  int setBT;

  h = LINUX_CAN_Open("/dev/pcan32",O_RDONLY);
  baud = LINUX_CAN_BTR0BTR1(h, 1000000);
  setBT = CAN_Init(h, CAN_BAUD_1M, CAN_INIT_TYPE_ST);
  
  int ret;
  
  int msgID, dataID;
  boost::array<int,12> data;
  TPCANRdMsg readMsg;
  cob_roboskin::roboskinCAN sendMsg;
  
  if (h != NULL)
  {         
      while(ros::ok())
      {	  
	  ret = LINUX_CAN_Read(h, &readMsg);
	  msgID = readMsg.Msg.ID;
	  //if (msgID == 1010)
	  //{	            
	      dataID = readMsg.Msg.DATA[0];
	      if (dataID == 64)
	      {
		for (int k = 0; k < 7; k ++)
		{
		    data[k] = readMsg.Msg.DATA[k+1];
		}
	      }
	      else
	      {
		for (int k = 7; k < 12; k++)
		{
		    data[k] = readMsg.Msg.DATA[k - 6];
		    if (k == 11)
		    {
		      sendMsg.id = msgID;
		      sendMsg.data = data;
		      roboskin_pub.publish(sendMsg);
		    }
		}	      
	      }	      
	  //}	  
      }        
  }
  
  
  else
  {   
      cout << "\ncould not open device";
  }

  return 0;
}
