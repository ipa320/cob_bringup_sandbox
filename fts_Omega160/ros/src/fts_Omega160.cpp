/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering	
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: 
 * ROS stack name: 
 * ROS package name: 
 * Description:
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *			
 * Author: 
 * Supervised by: 
 *
 * Date of creation: Jan 2012
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing 
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
#include <stdlib.h>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <fts_Omega160/fts.h>

// ROS service includes
//--

// external includes
#include <cob_generic_can/CanItf.h>
#include <cob_generic_can/CanPeakSysUSB.h>
#include <math.h>

//####################
//#### node class ####
class NodeClass
{
    //
    public:
	      
	ros::NodeHandle n;
	CanItf* can_itf;
 
    // topics to publish       
	ros::Publisher topicPub_FTSData;
	
	// CAN address of the sensor, set by the DIP-switch 
	int NetBoxAddress;
		
	// topics to subscribe, callback is called for new messages arriving
	//--
        
        // service servers
        //--
            
        // service clients
        //--
        
        // global variables
        
        // Constructor
        NodeClass()
        {	
			// Set CAN-adress of sensor device
			NetBoxAddress = 8; 
			// create a handle for this node, initialize node
			ros::NodeHandle private_nh_("~");
		
			// read CAN settings 
			can_itf = new CANPeakSysUSB("/home/roboskin/git/fts_Omega160/CanCtrl.ini");  
		
			// init topic publisher
			//topicPub_FTSData = n.advertise<fts_Omega160::fts>("FTSData",1);
			topicPub_FTSData = n.advertise<fts_Omega160::fts>("FTSData",10);
        }
        
        // Destructor
        ~NodeClass() 
        {
        }
	
        // topic callback functions 
        // function will be called when a new message arrives on a topic
		//--

        // service callback functions
        // function will be called when a service is querried
        //--
	 
        // other function declarations
		
		// init function resets bias of sensor
		int InitFTS(NodeClass NodeHandle)
		{
			CanMsg CMsgTr;	
		
			// send command  
			CMsgTr.m_iLen = 1;
			CMsgTr.m_iID = NetBoxAddress;
			
			int Err=0; 
			
			unsigned char cMsg[8];
	
			cMsg[0] = 0x04;
			cMsg[1] = 0x00;
			cMsg[2] = 0x00;
			cMsg[3] = 0x00;
			cMsg[4] = 0x00;
			cMsg[5] = 0x00;
			cMsg[6] = 0x00;
			cMsg[7] = 0x00;

			CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
			Err = NodeHandle.can_itf->transmitMsg(CMsgTr);
			 
			return Err; 
		}
		
		// sensor data request
		int GetFTSData(NodeClass NodeHandle, double* Fx, double* Fy, double* Fz, double* Mx, double* My, double* Mz)
		{
			int fx, fy, fz, mx, my, mz, state; 
			double scale = 1000; 
			double minForceNx = 0; 
			double minForceNy = 0; 
			double minForceNz = 15; 
			double minTorqueNM = 1;
			
			CanMsg CMsgTr;	
		
			// request data 
			CMsgTr.m_iLen = 1;
			CMsgTr.m_iID = NetBoxAddress;

			unsigned char cMsg[8];
	
			cMsg[0] = 0x01;
			cMsg[1] = 0x00;
			cMsg[2] = 0x00;
			cMsg[3] = 0x00;
			cMsg[4] = 0x00;
			cMsg[5] = 0x00;
			cMsg[6] = 0x00;
			cMsg[7] = 0x00;

			CMsgTr.set(cMsg[0], cMsg[1], cMsg[2], cMsg[3], cMsg[4], cMsg[5], cMsg[6], cMsg[7]);
			NodeHandle.can_itf->transmitMsg(CMsgTr);

			usleep(500);
			//usleep(50000);
		
	
			CMsgTr.m_iLen = 8;
			for (int i=0;i<4;i++) 
			{
				if (NodeHandle.can_itf->receiveMsg(&CMsgTr)==0){return -1;}
			
				switch (CMsgTr.m_iID)
				{
					case (9) : 
					{ 	// get x forces and torques
						fx = CMsgTr.getAt(0) | (CMsgTr.getAt(1) << 8) | (CMsgTr.getAt(2) << 16) | (CMsgTr.getAt(3) << 24); 
						mx = CMsgTr.getAt(4) | (CMsgTr.getAt(5) << 8) | (CMsgTr.getAt(6) << 16) | (CMsgTr.getAt(7) << 24);
					}
					case (10) : 
					{	// get y forces and torques
						fy = CMsgTr.getAt(0) | (CMsgTr.getAt(1) << 8) | (CMsgTr.getAt(2) << 16) | (CMsgTr.getAt(3) << 24); 
						my = CMsgTr.getAt(4) | (CMsgTr.getAt(5) << 8) | (CMsgTr.getAt(6) << 16) | (CMsgTr.getAt(7) << 24); 
					}
					case (11) : 
					{	 // get z forces and torques
						fz = CMsgTr.getAt(0) | (CMsgTr.getAt(1) << 8) | (CMsgTr.getAt(2) << 16) | (CMsgTr.getAt(3) << 24); 
						mz = CMsgTr.getAt(4) | (CMsgTr.getAt(5) << 8) | (CMsgTr.getAt(6) << 16) | (CMsgTr.getAt(7) << 24); 
					} 
					case (12) : 
					{	// get status of sensor (not used jet) 
					/*	state  = CMsgTr.getAt(0) | (CMsgTr.getAt(1) << 8) | (CMsgTr.getAt(2) << 16) | (CMsgTr.getAt(3) << 24); 
						if (state != 0x00)
						{ROS_ERROR("FTS-Sensor has an Error! Code: %i See ATI NetBox manual!", state);} 
					*/
					}
					default:
					{ // not used 
					}
	  			}
	  		}
		
			// calculation of SI-unit values with scalingfactor form ft-sensor
			// scalingfactor is 1000 (counts per force/torque) 
			// calibrated range of sensor is Fx,Fy = 2500N, Fz=6250N, Mx,My,Mz=400Nm
		
			
			// scale values to SI-units
			*Fx = fx/scale; //[N]
			*Fy = fy/scale;
			*Fz = fz/scale;
			*Mx = mx/scale; //[Nm]
			*My = my/scale;
			*Mz = mz/scale;
			
			// to opress small values 
			
			if (fabs(*Fx) < minForceNx){*Fx = 0;}
			if (fabs(*Fy) < minForceNy){*Fy = 0;}
			if (fabs(*Fz) < minForceNz){*Fz = 0;}
			
			if (fabs(*Mx) < minTorqueNM){*Mx = 0;}
			if (fabs(*My) < minTorqueNM){*My = 0;}
			if (fabs(*Mz) < minTorqueNM){*Mz = 0;}
			
			ROS_DEBUG("Fx: %f Fy: %f Fz: %f Mx: %f My: %f Mz: %f", *Fx, *Fy, *Fz, *Mx, *My, *Mz);  
			return 0; 
		}
};

/*---------------------------------------------------------------*/
//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    // initialize ROS, spezify name of node
    ros::init(argc, argv, "fts_Omega160");    
    NodeClass nodeClass;
	
	static double Fx=0, Fy=0, Fz=0, Mx=0, My=0, Mz=0; 
	
	fts_Omega160::fts msg; 

	//ros::Rate loop_rate(100);
	//ros::Rate loop_rate(2000);
	ros::Rate loop_rate(200);
	
	// init sensor
	if (nodeClass.InitFTS(nodeClass)<0)
	{ROS_INFO("Error on init FTS Node");}
	else 
	{ROS_INFO("Init of FTS ok!");}  
	
	while (ros::ok())
	{   
		if(nodeClass.GetFTSData(nodeClass, &Fx, &Fy, &Fz, &Mx, &My, &Mz)!=0)
		{	ROS_INFO("Error on recieving Data from FTS!"); }
		
		// transformation of sensor 
		msg.Fx= Fz;
		msg.Fy= Fy * 0.707107 + Fx * 0.707107;
		msg.Fz= Fy * 0.707107 - Fx * 0.707107;  
		msg.Mx= Mz;
		msg.My= My * 0.707107 + Mx * 0.707107;
		msg.Mz= My * 0.707107 - Mx * 0.707107;
    	
        nodeClass.topicPub_FTSData.publish(msg);
 
        ros::spinOnce();
 
        loop_rate.sleep();
	}

    return 0;
}


