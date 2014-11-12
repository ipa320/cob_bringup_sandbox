#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/ICCam.h"
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/ICCam.h"
#endif

using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr ipa_CameraSensors::CreateColorCamera_ICCam()
{
	return AbstractColorCameraPtr(new ICCam());
}

#ifdef __LINUX__

#include <highgui.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <math.h>
#include <assert.h>


ICCam::ICCam() 
{
	m_initialized = false;
	m_open = false;
	m_BufferSize = 1;

	m_unicapCamera = new UnicapCamera();
	m_unicapCamera->SetColorMode(3); 
}

ICCam::ICCam(int res)
{
	m_initialized = false;
	m_open = false;

	m_unicapCamera = new UnicapCamera(res);
	m_unicapCamera->SetColorMode(3); 
}

ICCam::~ICCam() 
{
	delete m_unicapCamera;
}

unsigned long ICCam::Init(std::string directory, int cameraIndex) 
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_IC;

	std::string iniFileNameAndPath = directory + "cameraSensorsIni.xml";
	if (LoadParameters(iniFileNameAndPath.c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ICCam::Init: Parsing xml configuration file failed." << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);	
	}

	m_initialized = true;

	return RET_OK;
}

unsigned long ICCam::Open() 
{
	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	if (m_unicapCamera->Open() < 0)
	{
		return RET_FAILED;
	}
	
	m_open = true;
	return RET_OK;
	
}

unsigned long ICCam::Close()
{
	if (!isOpen())
	{
		return (RET_OK);
	}//m_unicapCamera

	if (m_unicapCamera->Close() < 0)
	{
		return RET_FAILED;
	}
	else
	{
		m_open = false;
		return RET_OK;
	}
}

unsigned long ICCam::GetColorImage(cv::Mat* img, bool getLatestFrame) 
{
	if (m_unicapCamera->GetColorImage(img) < 0) 
	{
		return RET_FAILED;
	}
	else
	{
		return RET_OK;
	}
}

unsigned long ICCam::SaveParameters(const char* filename) 
{
	return RET_OK;
}	

unsigned long ICCam::LoadParameters(const char* filename, int cameraIndex)
{
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));

	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ICCam::LoadParams: Error while loading xml configuration file (Check filename and syntax of the file):\n" << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "ICCam::LoadParams: Parsing xml configuration file:\n" << filename << std::endl;

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
//	BEGIN LibCameraSensors->ICCam
//************************************************************************************
			// Tag element "ICCam" of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_ICCam = NULL;
			std::stringstream ss;
			ss << "ICCam_" << cameraIndex;
			p_xmlElement_Root_ICCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_ICCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->SerialNumber
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "SerialNumber" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "id", &m_serialNumber ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'id' of tag 'SerialNumber'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
				}
				else
				{
					std::cerr << "ICCam::LoadParameters: Can't find tag 'SerialNumber'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->CameraType
//************************************************************************************
				// Subtag element "CameraType" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "CameraType" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					const std::string* tmpString;
					const std::string tagName = "type";
					tmpString = p_xmlElement_Child->Attribute(tagName);
					if (tmpString == 0)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'type' of tag 'CameraType'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_cameraType = *tmpString;
					}
				}
				else
				{
					std::cerr << "ICCam::LoadParameters: Can't find tag 'CameraType'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->Resolution
//************************************************************************************
				/// Subtag element "Resolution" of Xml Inifile
				//p_xmlElement_Child = NULL;
				//p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "Resolution" );
				//if ( p_xmlElement_Child )
				//{
				//	// read and save value of attribute
				//	if ( p_xmlElement_Child->QueryValueAttribute( "xwidth", &m_resolution[0] ) != TIXML_SUCCESS)
				//	{
				//		std::cerr << "ICCam::LoadParameters: Can't find attribute 'xwidth' of tag 'Resolution'." << std::endl;
				//		return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
				//	}
				//	// read and save value of attribute
				//	if ( p_xmlElement_Child->QueryValueAttribute( "ywidth", &m_resolution[1] ) != TIXML_SUCCESS)
				//	{
				//		std::cerr << "ICCam::LoadParameters: Can't find attribute 'ywidth' of tag 'Resolution'." << std::endl;
				//		return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
				//	}
				//}
				//else
				//{
				//	std::cerr << "ICCam::LoadParameters: Can't find tag 'Resolution'." << std::endl;
				//	return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				//}
		
//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->IntrinsicParameters
//************************************************************************************
				// Subtag element "IntrinsicParameters" of Xml Inifile
				/*p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "IntrinsicParameters" );
				if ( p_xmlElement_Child )
				{
					double fx, fy, cx, cy;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "fx", &fx ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'fx' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "fy", &fy ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'fy' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "cx", &cx ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'cx' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "cy", &cy ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'cy' of tag 'IntrinsicParameters'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					SetIntrinsicParameters(fx, fy, cx, cy);	
				}
				else
				{
					std::cerr << "ICCam::LoadParameters: Can't find tag 'IntrinsicParameters'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->DistortionCoeffs
//************************************************************************************
				// Subtag element "DistortionCoeffs " of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "DistortionCoeffs" );
				if ( p_xmlElement_Child )
				{
					double k1, k2, p1, p2;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "k1", &k1 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'k1' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "k2", &k2 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'k2' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "p1", &p1 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'p1' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "p2", &p2 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'p2' of tag 'DistortionCoeffs '." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					SetDistortionParameters(k1, k2, p1, p2, 1024, 768);	
				}
				else
				{
					std::cerr << "ICCam::LoadParameters: Can't find tag 'DistortionCoeffs '." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->Translation
//************************************************************************************
				// Subtag element "IntrinsicParameters" of Xml Inifile
				CvMat* extrinsicTranslation = cvCreateMat(3, 1, CV_64FC1);
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "Translation" );
				if ( p_xmlElement_Child )
				{
					double x, y, z;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "x", &x ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x' of tag 'Translation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "y", &y ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'y' of tag 'Translation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "z", &z ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'z' of tag 'Translation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					cvmSet(extrinsicTranslation, 0, 0, x);
					cvmSet(extrinsicTranslation, 1, 0, y);
					cvmSet(extrinsicTranslation, 2, 0, z);
				}
				else
				{
					std::cerr << "ICCam::LoadParameters: Can't find tag 'Translation'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->Rotation
//************************************************************************************
				// Subtag element "IntrinsicParameters" of Xml Inifile
				CvMat* extrinsicRotation = cvCreateMat(3, 3, CV_64FC1);
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "Rotation" );
				if ( p_xmlElement_Child )
				{
					double x11, x12, x13;
					double x21, x22, x23;
					double x31, x32, x33;
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "x11", &x11 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x11' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x12", &x12 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x12' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x13", &x13 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x13' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x21", &x21 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x21' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x22", &x22 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x22' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x23", &x23 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x23' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x31", &x31 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x31' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x32", &x32 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x32' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "x33", &x33 ) != TIXML_SUCCESS)
					{
						std::cerr << "ICCam::LoadParameters: Can't find attribute 'x33' of tag 'Rotation'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					cvmSet(extrinsicRotation, 0, 0, x11);
					cvmSet(extrinsicRotation, 0, 1, x12);
					cvmSet(extrinsicRotation, 0, 2, x13);
					cvmSet(extrinsicRotation, 1, 0, x21);
					cvmSet(extrinsicRotation, 1, 1, x22);
					cvmSet(extrinsicRotation, 1, 2, x23);
					cvmSet(extrinsicRotation, 2, 0, x31);
					cvmSet(extrinsicRotation, 2, 1, x32);
					cvmSet(extrinsicRotation, 2, 2, x33);
				}
				else
				{
					std::cerr << "ICCam::LoadParameters: Can't find tag 'Rotation'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

				SetExtrinsicParameters(extrinsicRotation, extrinsicTranslation);
				cvReleaseMat(&extrinsicTranslation);
				cvReleaseMat(&extrinsicRotation);
			}


//************************************************************************************
//	END LibCameraSensors->ICCam
//************************************************************************************
			else 
			{
				std::cerr << "ICCam::LoadParameters: Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}*/
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ICCam::LoadParameters: Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	return RET_OK;
}
}

unsigned long ICCam::SetProperty(t_cameraProperty* cameraProperty)
{
	int ret = 0;
	switch (cameraProperty->propertyID)
	{
		case PROP_BRIGHTNESS:	
			ret = m_unicapCamera->SetProperty(0, cameraProperty->integerData); 		
			break;
		case PROP_WHITE_BALANCE_U:	
			ret = m_unicapCamera->SetProperty(4, cameraProperty->integerData); 		
			break;
		case PROP_HUE:	
			ret = m_unicapCamera->SetProperty(6, cameraProperty->integerData); 		
			break;
		case PROP_SATURATION:	
			ret = m_unicapCamera->SetProperty(7, cameraProperty->integerData); 		
			break;
		case PROP_GAMMA:	
			ret = m_unicapCamera->SetProperty(8, cameraProperty->integerData); 		
			break;
		case PROP_EXPOSURE_TIME:	
			ret = m_unicapCamera->SetProperty(9, cameraProperty->integerData); 		
			break;
		case PROP_GAIN:	
			ret = m_unicapCamera->SetProperty(10, cameraProperty->integerData); 		
			break;
		case PROP_OPTICAL_FILTER:	
			ret = m_unicapCamera->SetProperty(11, cameraProperty->integerData); 		
			break;
		case PROP_FRAME_RATE:	
			ret = m_unicapCamera->SetProperty(12, cameraProperty->integerData); 		
			break;
		case PROP_REGISTER:	
			ret = m_unicapCamera->SetProperty(13, cameraProperty->integerData); 		
			break;
		case PROP_TIMEOUT:	
			ret = m_unicapCamera->SetProperty(14, cameraProperty->integerData); 		
			break;
		case PROP_CAMERA_RESOLUTION:	
			ret = m_unicapCamera->SetProperty(15, cameraProperty->integerData); 		
			break;

		default: 				
			std::cout << "ICCam::SetProperty: Property " << cameraProperty->propertyID << " unspecified.";
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


unsigned long ICCam::GetProperty(t_cameraProperty* cameraProperty)
{
	cameraProperty->cameraResolution.xResolution = 1024;
	cameraProperty->cameraResolution.yResolution = 768;
	return RET_OK;
}

unsigned long ICCam::SetPropertyDefaults()
{
	unsigned long ret = 0;
	t_cameraProperty cameraProperty;
	
	cameraProperty.integerData = 0;
	cameraProperty.propertyID = PROP_BRIGHTNESS;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	/*cameraProperty.integerData = 160;
	cameraProperty.propertyID = PROP_AUTO_EXPOSURE;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);*/

	cameraProperty.integerData = 8;
	cameraProperty.propertyID = PROP_SHARPNESS;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	/*cameraProperty.integerData = 8;
	cameraProperty.propertyID = PROP_WHITE_BALANCE_MODE;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);*/

	cameraProperty.integerData = 59;
	cameraProperty.propertyID = PROP_WHITE_BALANCE_U;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 46;
	cameraProperty.propertyID = PROP_WHITE_BALANCE_V;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 45;
	cameraProperty.propertyID = PROP_HUE;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 27;
	cameraProperty.propertyID = PROP_SATURATION;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);
	
	cameraProperty.integerData = 3;
	cameraProperty.propertyID = PROP_GAMMA;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 1068;
	cameraProperty.propertyID = PROP_SHUTTER;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 0;
	cameraProperty.propertyID = PROP_GAIN;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 1;
	cameraProperty.propertyID = PROP_OPTICAL_FILTER;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 15;
	cameraProperty.propertyID = PROP_FRAME_RATE;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 0;
	cameraProperty.propertyID = PROP_REGISTER;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	cameraProperty.integerData = 1;
	cameraProperty.propertyID = PROP_TIMEOUT;
	cameraProperty.propertyType = TYPE_INTEGER;
	ret |= SetProperty(&cameraProperty);

	return ret;

}

unsigned long ICCam::PrintCameraInformation()
{
	if (!isOpen())
	{
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	if (m_unicapCamera == 0 || m_unicapCamera->PrintCameraInformation() < 0)
	{
		return RET_FAILED;
	}

	return RET_OK;
}

/*unsigned long ICCam::LoadParameters(const char* filename, int cameraIndex)
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}*/

unsigned long ICCam::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}

	// testing of the property functions
	return RET_OK;
}

#else
//****************************************************************************************
// WINDOWS
//****************************************************************************************

ICCam::ICCam()
{
	m_initialized = false;
	m_open = false;
	m_BufferSize = 1;

	m_grabber = 0;
}

ICCam::~ICCam()
{
	
}

unsigned long ICCam::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_IC;

	std::string iniFileNameAndPath = directory + "cameraSensorsIni.xml";
	if (LoadParameters(iniFileNameAndPath.c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - ICCam::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);	
	}

	if (DShowLib::InitLibrary(m_serialNumber.c_str()) == 0)
	{
		std::cerr << "ICCam::Init:" << std::endl;
		std::cerr << "\t ... Initializing library failed" << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);	
	}

	m_initialized = true;

	return RET_OK;

}

unsigned long ICCam::Open()
{
	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	m_grabber = new DShowLib::Grabber();
	//m_grabber->showDevicePage();
	m_grabber->openDev(m_cameraType); //"DBx 31AF03" or "DFx 41F02";

	if( m_grabber->isDevValid() )  // Check if there is a valid device.
	{
		m_pSink = DShowLib::FrameHandlerSink::create(DShowLib::eRGB24, 1);
		m_pSink->setSnapMode(true);
		m_grabber->setSinkType( m_pSink );	// Set the sink
		
		// Prepare the live mode, to get the output size of the sink.
		if(!m_grabber->prepareLive(false))
		{
			std::cerr << "ERROR - ICCam::Open:" << std::endl;
			std::cerr << "\t ... Could not render the VideoFormat into a eY800 sink" << std::endl;
			Close();
			return RET_FAILED;
		}

		m_pSink->getOutputFrameType(m_Info);
		m_ColorImageResolution = cvSize((int)m_Info.dim.cx,(int) m_Info.dim.cy); // cvSize(1024, 768)

		m_grabber->startLive(false);	// Start the live video.

		if(!(m_ColorImageResolution.width==ICCAM_COLUMNS && m_ColorImageResolution.height==ICCAM_ROWS))
		{
			std::cerr << "ERROR - ICCam::Open:" << std::endl;
			std::cerr << "\t ... Resolution differs from expected value of " << ICCAM_COLUMNS << "x" << ICCAM_ROWS << std::endl;
			Close();
			return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
		}
	}
	else
	{
		std::cerr << "ERROR - ICCam::Open:" << std::endl;
		std::cerr << "\t ... No valid device for color camera selected (" << m_cameraType << ")." << std::endl;
		Close();
		return (RET_FAILED | RET_OPEN_CAMERA_FAILED);
	}

	std::cout << "**************************************************" << std::endl;
	std::cout << "ICCam::Open: AVT Pike 145C camera device OPEN" << std::endl;
	std::cout << "**************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}

unsigned long ICCam::Close() 
{
	if (!isOpen())
	{
		return (RET_OK);
	}

	m_grabber->stopLive();

	if (m_grabber)
	{
		delete m_grabber;
		m_grabber = 0;
	}

	m_open = false;

	return RET_OK;
}

unsigned long ICCam::GetColorImage(cv::Mat* colorImage, bool getLatestFrame)
{
	CV_Assert (colorImage != 0);

	if (isOpen())
	{
		
		_DSHOWLIB_NAMESPACE::tErrorEnum ret = m_pSink->snapImages(1);
		if (ret != _DSHOWLIB_NAMESPACE::eNOERROR) {
			return RET_FAILED;
		}

		DShowLib::MemBufferCollection::tMemBufferPtr buffer = m_pSink->getLastAcqMemBuffer();

		if (buffer != 0) {
			cv::Mat image(m_ColorImageResolution, CV_8UC3, (char*) buffer->getPtr());
			colorImage->create(m_ColorImageResolution, CV_8UC3);
			cv::flip(image, (*colorImage), 0);
		}

		return RET_OK;
	}
	else
	{
		return RET_FAILED;
	}
	return RET_OK;
}

unsigned long ICCam::SaveParameters(const char* filename) 
{
	return RET_FAILED;
}

unsigned long ICCam::SetProperty(t_cameraProperty* cameraProperty) 
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
		case PROP_VIDEO_FORMAT:
			/*{
				// Not yet working -> See eample Pixelformat foa a running implementation
				Grabber::tVidFmtListPtr pVidFmtList = m_grabber->getAvailableVideoFormats();
				switch (cameraProperty->videoFormat)
				{			
					case BY8:	
						if (m_grabber->setVideoFormat( pVidFmtList->at(0) ))
						{ 
							std::cout << "ICCam::SetProperty: Video format " << pVidFmtList->at(0).toString() << " set.\n";
						}
						else
						{
							std::cout << "ICCam::SetProperty: Setting video format " << pVidFmtList->at(0).toString() << " failed.\n";
							return RET_FAILED;
						}
						break;
					case UYVY:	
						if (m_grabber->setVideoFormat( pVidFmtList->at(1) )) 
						{
							std::cout << "ICCam::SetProperty: Video format " << pVidFmtList->at(1).toString() << " set.\n";
						}
						else
						{
							std::cout << "ICCam::SetProperty: Setting video format " << pVidFmtList->at(1).toString() << " failed.\n";
							return RET_FAILED;
						}
						break;
					case Y800:	
						if (m_grabber->setVideoFormat( pVidFmtList->at(2) ))
						{
							std::cout << "ICCam::SetProperty: Video format " << pVidFmtList->at(2).toString() << " set.\n";
						}
						else
						{
							std::cout << "ICCam::SetProperty: Setting video format " << pVidFmtList->at(2).toString() << " failed.\n";
							return RET_FAILED;
						}
						break;
					default: 				
						std::cout << "ICCam::SetProperty: Video format unspecified.";
						ret = -1; 
						break;
				}
			}*/
			break;
		
		default: 				
			std::cout << "ICCam::SetProperty: Property " << cameraProperty->propertyID << " unspecified.";
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

unsigned long ICCam::SetPropertyDefaults() 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long ICCam::GetProperty(t_cameraProperty* cameraProperty) 
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
		case PROP_EXPOSURE_TIME:	
			// implement me 		
			break;
		case PROP_GAIN:	
			// implement me 		
			break;
		case PROP_CAMERA_RESOLUTION:
			{
				// Implement me correctly by asking the camera
				cameraProperty->cameraResolution.xResolution = 1024;
				cameraProperty->cameraResolution.yResolution = 768;
			}
			break;
		case PROP_VIDEO_FORMAT:
		// Specifies the format of the images that are sent by the camera to the sink.
		// Depending on the image foramt specified for the sink, the received images
		// are converted to another format (i.e. eRGB24)
			/*{
				std::string videoFormat = m_grabber->getVideoFormat().toString();
				std::cout << "ICCam::GetProperty: Current video format is " << videoFormat << "." << std::endl;

				// like Y800, but specifies that color interpolation is switched off and
				// RAW image is transferred to the sink.
				if (videoFormat == "BY8 (1024x768)")
				{
					cameraProperty->videoFormat = BY8;
					cameraProperty->propertyType = TYPE_VIDEO_FORMAT;
					return RET_OK;
				}

				// YUV color format. 16 bit per pixel.
				// See www.fourcc.org for more details.
				else if (videoFormat == "UYVY (1024x768)")
				{
					cameraProperty->videoFormat = UYVY;
					cameraProperty->propertyType = TYPE_VIDEO_FORMAT;
					return RET_OK;
				}

				// 256 graylevels per pixel. Image is received either by interpolating the color values and
				// return the luminance value (for low cost cameras interpolation is alwys on), or if possible
				// switch off interpolation and return the raw image without interpolation.
				else if (videoFormat == "Y800 (1024x768)")
				{
					cameraProperty->videoFormat = Y800;
					cameraProperty->propertyType = TYPE_VIDEO_FORMAT;
					return RET_OK;
				}
				else
				{
					std::cout << "ICCam::GetProperty: Video format " << videoFormat << " not recognized.\n"; 
					return RET_FAILED;
				}
				
			}*/
			break;
		
		default: 				
			std::cout << "ICCam::SetProperty: Property " << cameraProperty->propertyID << " unspecified.";
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


unsigned long ICCam::PrintCameraInformation()
{
	if (!isOpen())
	{
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}
	// Print device name
	std::cout << "Device name: " << m_grabber->getDev().toString() << std::endl;
	std::cout << "Current video format: " << m_grabber->getVideoFormat().toString() << ".\n";
	std::cout << "Current frame rate: " << m_grabber->getFrameRate() << std::endl;

	// Print available video formats
	if( m_grabber->isVideoNormAvailableWithCurDev() )
	{
		std::cout << "\nSupported Video Norms and Formats: \n";
		Grabber::tVidNrmListPtr pVidNrmList = m_grabber->getAvailableVideoNorms();
		// List the available video formats for each video norm (PAL, NTSC,...)
		unsigned int nrmCounter = 0;
		for( Grabber::tVidNrmList::iterator itNrm = pVidNrmList->begin(); 
			itNrm != pVidNrmList->end(); 
			++itNrm )
		{
			std::cout << "Video Norm [" << nrmCounter++ << "]: " << itNrm->toString() << std::endl;
			m_grabber->setVideoNorm( pVidNrmList->at( nrmCounter ) );
			
			// Now retrieve the video formats.
			Grabber::tVidFmtListPtr pVidFmtList = m_grabber->getAvailableVideoFormats();
			if( pVidFmtList == 0 ) 
			{
				std::cerr << "ICCam::PrintCameraInformation: " << m_grabber->getLastError().toString() << std::endl;
				return RET_FAILED;
			}				

			unsigned int fmtCounter = 0;
			// List the available video formats.
			for( Grabber::tVidFmtList::iterator itFmt = pVidFmtList->begin(); 
				  itFmt != pVidFmtList->end(); 
				  ++itFmt )
			{
				std::cout << "\t[" << fmtCounter++ << "] " << itFmt->toString() << std::endl;
			}

			std::cout << std::endl << std::endl;

		}

	}
	else
	{
		// If the current video capture device does not support video norms,
		// the available video formats can be retrieved immediately.
		std::cout << "\nSupported Video Formats: \n";
			
		Grabber::tVidFmtListPtr pVidFmtList = m_grabber->getAvailableVideoFormats();
			
		if( pVidFmtList == 0 ) // No video formats available?
		{
			std::cerr << "ICCam::PrintCameraInformation: " << m_grabber->getLastError().toString() << std::endl;
			return RET_FAILED;
		}
		else
		{
			unsigned int fmtCounter = 0;
			// List the available video formats.
			for( Grabber::tVidFmtList::iterator itFmd = pVidFmtList->begin(); 
				itFmd != pVidFmtList->end(); 
				++itFmd )
			{
				std::cout << "\t[" << fmtCounter++ << "] " << itFmd->toString() << std::endl;
			}
		}
	}


	return RET_OK;
}

unsigned long ICCam::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}

	// Testing of the property functions
	/*std::cout << "ICCam::TestCamera: checking SetProperty()..." << std::endl;
	t_cameraProperty cameraProperty;
	t_videoFormat videoFormat = UYVY;
	cameraProperty.propertyID = PROP_VIDEO_FORMAT;
	cameraProperty.propertyType = TYPE_VIDEO_FORMAT;
	cameraProperty.videoFormat = videoFormat;
	if (SetProperty(&cameraProperty) & RET_FAILED)
	{
		std::cout << "ICCam::TestCamera: checking SetProperty()...          FAILED" << std::endl;
		return (RET_FAILED | RET_SET_PROPERTY_FAILED);
	}
	std::cout << "ICCam::TestCamera: checking SetProperty()...          OK" << std::endl;
	std::cout << std::endl;

	std::cout << "ICCam::TestCamera: checking GetProperty()..." << std::endl;
	t_cameraProperty cameraPropertyB;
	cameraPropertyB.propertyID = PROP_VIDEO_FORMAT;
	if (GetProperty(&cameraPropertyB) & RET_FAILED)
	{
		std::cout << "ICCam::TestCamera: checking GetProperty()...          FAILED" << std::endl;
		return (RET_FAILED | RET_GET_PROPERTY_FAILED);
	}
	else if (cameraPropertyB.propertyType != TYPE_VIDEO_FORMAT || 
			 cameraPropertyB.videoFormat != videoFormat)
	{
		std::cout << "ICCam::TestCamera: Received wrong parameters." << std::endl;
		std::cout << "ICCam::TestCamera: checking GetProperty()...          FAILED" << std::endl;
		return (RET_FAILED | RET_GET_PROPERTY_FAILED);
	}
	std::cout << "ICCam::TestCamera: checking GetProperty()...          OK" << std::endl;*/
	return RET_OK;
}

unsigned long ICCam::LoadParameters(const char* filename, int cameraIndex)
{
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));

	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file" << std::endl;
		std::cerr << "\t ... (Check filename and syntax of the file):\n" << filename << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - ICCam::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << filename << "'" << std::endl;

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
//	BEGIN LibCameraSensors->ICCam
//************************************************************************************
			// Tag element "ICCam" of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_ICCam = NULL;
			std::stringstream ss;
			ss << "ICCam_" << cameraIndex;
			p_xmlElement_Root_ICCam = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_ICCam )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->SerialNumber
//************************************************************************************
				// Subtag element "SerialNumber" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "SerialNumber" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "id", &m_serialNumber ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'id' of tag 'SerialNumber'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
				}
				else
				{
					std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
					std::cerr << "\t ... Can't find tag 'SerialNumber'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->ICCam->CameraType
//************************************************************************************
				// Subtag element "CameraType" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_ICCam->FirstChildElement( "CameraType" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					const std::string* tmpString;
					const std::string tagName = "type";
					tmpString = p_xmlElement_Child->Attribute(tagName);
					if (tmpString == 0)
					{
						std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'type' of tag 'CameraType'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_cameraType = *tmpString;
					}
				}
				else
				{
					std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CameraType'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->ICCam
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - ICCam::LoadParams:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}

	
	return RET_OK;
}

#endif // __LINUX__

