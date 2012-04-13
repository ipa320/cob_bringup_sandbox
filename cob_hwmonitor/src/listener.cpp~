#include "ros/ros.h"
#include "std_msgs/String.h"
#include "cob_hwmonitor/hw_msg.h"

using namespace std;

void chatterCallback(const cob_hwmonitor::hw_msg msg)
{
  cout << "\n";
  cout << "\t\tCurrent Temp\t\t\t\tMinimum Temp\t\t\t\tMaximum Temp\n";
  cout << "Sensor 1:\t" << (int)msg.temp_1_curr << "\t\t\t\t\t" << (int)msg.temp_1_min << "\t\t\t\t\t" << (int)msg.temp_1_max << "\n";
  cout << "Sensor 2:\t" << (int)msg.temp_2_curr << "\t\t\t\t\t" << (int)msg.temp_2_min << "\t\t\t\t\t" << (int)msg.temp_2_max << "\n";
  cout << "Sensor 3:\t" << (int)msg.temp_3_curr << "\t\t\t\t\t" << (int)msg.temp_3_min << "\t\t\t\t\t" << (int)msg.temp_3_max << "\n";
  cout << "Sensor 4:\t" << (int)msg.temp_4_curr << "\t\t\t\t\t" << (int)msg.temp_4_min << "\t\t\t\t\t" << (int)msg.temp_4_max << "\n";
  cout << "Sensor 5:\t" << (int)msg.temp_5_curr << "\t\t\t\t\t" << (int)msg.temp_5_min << "\t\t\t\t\t" << (int)msg.temp_5_max << "\n";
  cout << "Sensor 6:\t" << (int)msg.temp_6_curr << "\t\t\t\t\t" << (int)msg.temp_6_min << "\t\t\t\t\t" << (int)msg.temp_6_max << "\n";
  cout << "\n";
  cout << "\t\tCurrent Voltage\t\t\t\tMinimum Voltage\t\t\t\tMaximum Voltage\n";
  cout << "Akku:\t\t" << (int)msg.akku_voltage_curr << "\t\t\t\t\t" << (int)msg.akku_voltage_min << "\t\t\t\t\t" << (int)msg.akku_voltage_max << "\n";
  cout << "Hals-Motor:\t" << (int)msg.hals_motor_voltage_curr << "\t\t\t\t\t" << (int)msg.hals_motor_voltage_min << "\t\t\t\t\t" << (int)msg.hals_motor_voltage_max << "\n";
  cout << "Hals-Logik:\t" << (int)msg.hals_logik_voltage_curr << "\t\t\t\t\t" << (int)msg.hals_logik_voltage_min << "\t\t\t\t\t" << (int)msg.hals_logik_voltage_min << "\n";
  cout << "Tablett-Logik:\t" << (int)msg.tablett_logik_voltage_curr << "\t\t\t\t\t" << (int)msg.tablett_logik_voltage_min << "\t\t\t\t\t" << (int)msg.tablett_logik_voltage_max << "\n";
  cout << "Arm-Logik:\t" << (int)msg.arm_logik_voltage_curr << "\t\t\t\t\t" << (int)msg.arm_logik_voltage_min << "\t\t\t\t\t" << (int)msg.arm_logik_voltage_max << "\n";
  cout << "Tablett-Motor:\t" << (int)msg.tablett_motor_voltage_curr << "\t\t\t\t\t" << (int)msg.tablett_motor_voltage_min << "\t\t\t\t\t" << (int)msg.tablett_motor_voltage_max << "\n";
  cout << "\n";
  cout << "\t\tCurrent Current\t\t\t\tMinimum Current\t\t\t\tMaximum Current\n";
  cout << "Hals-Motor:\t" << (int)msg.hals_motor_current_curr << "\t\t\t\t\t" << (int)msg.hals_motor_current_min << "\t\t\t\t\t" << (int)msg.hals_motor_current_max << "\n";
  cout << "Hals-Logik:\t" << (int)msg.hals_logik_current_curr << "\t\t\t\t\t" << (int)msg.hals_logik_current_min << "\t\t\t\t\t" << (int)msg.hals_logik_current_max << "\n";
  cout << "Tablett-Logik:\t" << (int)msg.tablett_logik_current_curr << "\t\t\t\t\t" << (int)msg.tablett_logik_current_min << "\t\t\t\t\t" << (int)msg.tablett_logik_current_max << "\n";
  cout << "Arm-Logik:\t" << (int)msg.arm_logik_current_curr << "\t\t\t\t\t" << (int)msg.arm_logik_current_min << "\t\t\t\t\t" << (int)msg.arm_logik_current_max << "\n";
  cout << "Tablett-Motor:\t" << (int)msg.tablett_motor_current_curr << "\t\t\t\t\t" << (int)msg.tablett_motor_current_min << "\t\t\t\t\t" << (int)msg.tablett_motor_current_max << "\n";
  cout << "\n\n";
}

int main(int argc, char **argv)

{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("hwmonitor", 1000, chatterCallback);
  ros::spin();
  
  return 0;
}

