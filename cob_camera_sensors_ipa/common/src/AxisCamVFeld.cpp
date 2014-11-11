#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/AxisCamVFeld.h"

	#include "tinyxml.h"
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/AxisCamVFeld.h"
#endif



#ifndef __LINUX__

using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr ipa_CameraSensors::CreateColorCamera_AxisCam()
{
	return AbstractColorCameraPtr(new AxisCam());
}

AxisCam::AxisCam()
{
	m_initialized = false;
	m_open = false;
	m_BufferSize = 1;
	
	m_IPCamera = new IPCamera();
}

AxisCam::~AxisCam()
{
	if (isOpen())
	{
		Close();
	}
}

unsigned long AxisCam::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_AXIS;

	std::string iniFileNameAndPath = directory + "cameraSensorsIni.xml";
	if (LoadParameters(iniFileNameAndPath.c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - AxisCam::Init" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);	
	}

	std::string sInterface = "";
	std::string sIP = "";
	m_ColorCameraParameters.m_Interface.clear(); // Clear flags
	m_ColorCameraParameters.m_Interface.seekg(0); // Set Pointer to position 0 within stringstream
	m_ColorCameraParameters.m_Interface >> sInterface;

	if (sInterface == "ETHERNET")
	{
		m_ColorCameraParameters.m_IP.clear(); // Clear flags
		m_ColorCameraParameters.m_IP.seekg(0); // Set Pointer to position 0 within stringstream
		m_ColorCameraParameters.m_IP >> sIP;
	}
	else
	{
		std::cerr << "ERROR - AxisCam::Init:" << std::endl;
		std::cerr << "\t ... Unknown interface camera '" << sInterface << "'"<< std::endl;
		std::cerr << "\t ... Aborting camera initialization\n";
		return RET_FAILED;
	}

	// Setup HTTP client
	//#define CAMERA_AXIS_COMPRESSION_LOWEST    330000 // bytes
	//#define CAMERA_AXIS_COMPRESSION_LOW       32000 // bytes
	//#define CAMERA_AXIS_COMPRESSION_MEDIUM    15000 // bytes
	//#define CAMERA_AXIS_COMPRESSION_HIGH      9000 // bytes
	int m_bufferSize = 9000;
	// 192.168.0.2 for tracking camera
	// 192.168.0.90 for vislok camera
	std::string sURL = "http://";
	sURL += sIP;
	sURL += "/axis-cgi/mjpg/video.cgi?resolution=640x480";
	const char* cURL = sURL.c_str();	
	if (m_IPCamera->Init(m_bufferSize, cURL) & RET_FAILED)
	{
		std::cerr << "ERROR - AxisCam::Init" << std::endl;
		std::cerr << "\t ... Error while initializing IP camera." << std::endl;
		return RET_FAILED;	
	}
	
	m_initialized = true;
	return RET_OK;
}

unsigned long AxisCam::Open()
{
	if (!isInitialized())
	{
		std::cerr << "ERROR - AxisCam::Open" << std::endl;
		std::cerr << "\t ... Camera not initialized.\n";
		return RET_FAILED;
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	if (m_IPCamera->Open() & RET_FAILED)
	{
		std::cerr << "ERROR - AxisCam::Open" << std::endl;
		std::cerr << "\t ... Error while opening IP camera." << std::endl;
		return (RET_FAILED | RET_OPEN_CAMERA_FAILED);	
	}

	std::cout << "*************************************************" << std::endl;
	std::cout << "AxisCam::Open: Axis camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;
	return RET_OK;
}

unsigned long AxisCam::Close() 
{
	if (!isOpen())
	{
		return (RET_OK);
	}

	if (m_IPCamera)
	{
		delete m_IPCamera;
	}

	m_open = false;

	return RET_OK;
}

unsigned long AxisCam::GetColorImage(cv::Mat* colorImage, bool getLatestImage)
{
	if (!isInitialized())
	{
		std::cerr << "ERROR - AxisCam::GetColorImage" << std::endl;
		std::cerr << "\t ... Camera not initialized." << std::endl;
		return RET_FAILED;
	}

	if (!isOpen())
	{
		std::cerr << "ERROR - AxisCam::GetColorImage" << std::endl;
		std::cerr << "\t ... Camera not open." << std::endl;
		return RET_FAILED;
	}

	while (m_IPCamera->GetColorImage(colorImage) & RET_FAILED)
	{
		// NOT a continuous polling, as condition variable is used internally
		// within class IPCamera to wait for an arriving picture
		
		//std::cout << "AxisCam::GetColorImage: Waiting." << std::endl;
	}
	
	return RET_OK;
}

unsigned long AxisCam::SaveParameters(const char* filename) {return RET_FUNCTION_NOT_IMPLEMENTED;}

unsigned long AxisCam::LoadParameters(const char* filename, int cameraIndex)
{
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));

	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - AxisCam::LoadParameters" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file" << std::endl;
		std::cerr << "\t ... (Check filename and syntax):" << std::endl;
		std::cerr << "\t ... " << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - AxisCam::LoadParameters" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... " << filename << std::endl;

	if ( p_configXmlDocument )
	{

//************************************************************************************
//	BEGIN LibCameraSensors
//************************************************************************************
		// Tag element "LibCameraSensors" of Xml Inifile		
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "LibCameraSensors" );
		if ( p_xmlElement_Root )
		{

//************************************************************************************
//	BEGIN LibCameraSensors->AxisCam
//************************************************************************************
			// Tag element "AxisCam" of Xml Inifile		
			TiXmlElement* p_xmlElement_Root_ICCam = NULL;
			std::stringstream ss;
			ss << "AxisCam_" << cameraIndex;
			p_xmlElement_Root_ICCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_ICCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->AxisCam->Interface
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "Interface" );
				std::string tempString;
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - AxisCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Interface'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					
					if (tempString == "ETHERNET")
					{
						m_ColorCameraParameters.m_Interface.str( " " );	// Clear stringstream
						m_ColorCameraParameters.m_Interface.clear();		// Reset flags
						m_ColorCameraParameters.m_Interface << tempString;
						// read and save value of attribute
						if ( p_xmlElement_Child->QueryValueAttribute( "ip", &tempString ) != TIXML_SUCCESS)
						{
							std::cerr << "ERROR - AxisCam::LoadParameters:" << std::endl;
							std::cerr << "\t ... Can't find attribute 'ip' of tag 'Interface'." << std::endl;
							return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
						}
						m_ColorCameraParameters.m_IP.str( " " );	// Clear stringstream
						m_ColorCameraParameters.m_IP.clear();		// Reset flags
						m_ColorCameraParameters.m_IP << tempString;
					}
					else
					{
						std::cerr << "ERROR - AxisCam::LoadParameters:" << std::endl;
						std::cerr << "\t ... Interface " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - AxisCam::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Interface'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->AxisCam
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - AxisCam::LoadParameters" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - AxisCam::LoadParameters" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	return RET_OK;
}

unsigned long AxisCam::SetProperty(t_cameraProperty* cameraProperty) 
{
	if (!isOpen())
	{
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int ret = 0;
	switch (cameraProperty->propertyID)
	{
		case PROP_BRIGHTNESS:	
			// implement me
			break;
		case PROP_EXPOSURE_TIME:	
			// implement me	
			break;
		case PROP_WHITE_BALANCE_U:	
			// implement me 	
			break;
		case PROP_WHITE_BALANCE_V:	
			// implement me 	
			break;
		case PROP_HUE:	
			// implement me 		
			break;
		case PROP_SATURATION:	
			// implement me 	
			break;
		case PROP_GAMMA:	
			// implement me  		
			break;
		case PROP_GAIN:	
			// implement me 		
			break;
		case PROP_VIDEO_ALL:
			// implement me 
			break;
		
		default: 				
			std::cerr << "ERROR - AxisCam::SetProperty" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			ret = -1; 
			break;

	}

	if (ret < 0)
	{
		return RET_FAILED;
	}
	else
	{
		return RET_OK;
	}
}

unsigned long AxisCam::SetPropertyDefaults() 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long AxisCam::GetProperty(t_cameraProperty* cameraProperty) 
{
	if (!isOpen())
	{
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int ret = 0;
	switch (cameraProperty->propertyID)
	{
		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;
		case PROP_BRIGHTNESS:	
			// implement me
			break;
		case PROP_EXPOSURE_TIME:	
			// implement me	
			break;
		case PROP_WHITE_BALANCE_U:	
			// implement me 	
			break;
		case PROP_WHITE_BALANCE_V:	
			// implement me 	
			break;
		case PROP_HUE:	
			// implement me 		
			break;
		case PROP_SATURATION:	
			// implement me 	
			break;
		case PROP_GAMMA:	
			// implement me  		
			break;
		case PROP_GAIN:	
			// implement me 		
			break;
		case PROP_CAMERA_RESOLUTION:
			// this is just a hack. Implement me correctly by asking the camera
			cameraProperty->cameraResolution.xResolution = 640;
			cameraProperty->cameraResolution.yResolution = 480;
			break;
		case PROP_VIDEO_FORMAT:
			// implement me 
			break;
		
		default: 				
			std::cerr << "ERROR - AxisCam::GetProperty" << std::endl;
			std::cerr << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			ret = -1; 
			break;

	}

	if (ret < 0)
	{
		return RET_FAILED;
	}
	else
	{
		return RET_OK;
	}
}


unsigned long AxisCam::PrintCameraInformation()
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long AxisCam::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}
	return RET_OK;
}

#endif
