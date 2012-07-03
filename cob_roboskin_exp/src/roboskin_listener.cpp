#include "ros/ros.h"
#include "cob_roboskin/roboskinCAN.h"
#include "cob_srvs/Trigger.h"


using namespace std;

class Stopclient
{
	public:
	  ros::NodeHandle n;
	  ros::ServiceClient client;
	  void chatterCallback(const cob_roboskin::roboskinCAN msg);

};
	
void Stopclient::chatterCallback(const cob_roboskin::roboskinCAN msg)
{

  for (int i = 0; i < 12; i++)
  {
		if (msg.data[i] < 120){
			cout << "ID: " << msg.id << "\t";
			for (int i = 0; i < 12; i++)
		  	{
				cob_srvs::Trigger req;
				client = n.serviceClient<cob_srvs::Trigger>("/arm_controller/stop"); 
                		client.call(req);
				cout << msg.data[i] << " ";
			}
				cout << "\n";
	  }

}
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::init(argc, argv, "Stopclient");
  Stopclient my_cvi = Stopclient();
  //ros::NodeHandle n;
 
  ros::Subscriber sub = my_cvi.n.subscribe("roboskin", 1000, &Stopclient::chatterCallback, &my_cvi);
  ros::spin();

  return 0;
}

