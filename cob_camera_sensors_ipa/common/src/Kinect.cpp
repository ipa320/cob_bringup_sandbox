#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/Kinect.h"	
	#include "cob_vision_utils/GlobalDefines.h"

	#include "tinyxml.h"
	#include <fstream>
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/Kinect.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif



using namespace ipa_CameraSensors;
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )
#define IPA_CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))
#define XN_SXGA_X_RES 1280
#define XN_SXGA_Y_RES 1024
#define XN_VGA_X_RES 640
#define XN_VGA_Y_RES 480

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_Kinect()
{
	return AbstractRangeImagingSensorPtr(new Kinect());
}

Kinect::Kinect()
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;
	
	//m_CoeffsInitialized = false;
}

Kinect::~Kinect()
{
	if (isOpen())
	{
		/*
		m_context.StopGeneratingAll();
		m_context.Shutdown ();
		*/
		Close();
	}
}


unsigned long Kinect::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return ipa_Utils::RET_OK;
	}

	//XnStatus retVal = XN_STATUS_OK;

	openni::Status retVal = openni::STATUS_OK;

	//retVal = m_context.Init(); 
	retVal = openni::OpenNI::initialize();
	
	//if (retVal !=XN_STATUS_OK)
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot init openNI" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	retVal = m_device.open(openni::ANY_DEVICE);

	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot open device" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Load SR parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Create a DepthGenerator node 
	//retVal = m_depth_generator.Create(m_context);
	retVal = m_vs_d.create(m_device, openni::SENSOR_DEPTH);
	
	//if (retVal !=XN_STATUS_OK)
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot create depth image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Create a ImageGenerator node 
	//retVal = m_image_generator.Create(m_context);
	retVal = m_vs_rgb.create(m_device, openni::SENSOR_COLOR);

	//if (retVal !=XN_STATUS_OK)
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot create color image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Create a IR Stream 
	retVal = m_vs_ir.create(m_device, openni::SENSOR_IR);

	//if (retVal !=XN_STATUS_OK)
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot create IR image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Set video format
	//XnMapOutputMode m_output_mode;

	/*
	//Debug: show supported video mode
	openni::SensorType st = openni::SENSOR_DEPTH;
	std::cout << "Info - Depth Sensor: Supported Video Mode:" << std::endl; 
	for (int i=0; i< m_device.getSensorInfo(st)->getSupportedVideoModes().getSize();i++)
	{
		std::cout << "Video Mode " << i << std::endl; 
		std::cout << "\t...FPS: " << m_device.getSensorInfo(st)->getSupportedVideoModes()[i].getFps() << std::endl;
		std::cout << "\t...Pixel Format: " << m_device.getSensorInfo(st)->getSupportedVideoModes()[i].getPixelFormat() << std::endl;
		std::cout << "\t... Resolution: (" << m_device.getSensorInfo(st)->getSupportedVideoModes()[i].getResolutionX() << ","
			<< m_device.getSensorInfo(st)->getSupportedVideoModes()[i].getResolutionY() << ")" << std::endl << std::endl;
	}
	*/

	switch (m_ColorCamVideoFormat)
	{
	case SXGA:
		//m_output_mode.nFPS = 30;
		//m_output_mode.nXRes = XN_SXGA_X_RES;
		//m_output_mode.nYRes = XN_SXGA_Y_RES;
		m_output_mode.setFps(30);
		m_output_mode.setResolution(XN_SXGA_X_RES,XN_SXGA_Y_RES); //Resolution of SXGA : 1280*1024
		std::cout << "INFO - ColorCamVideoFormat: SXGA" << std::endl;
		break;
	case VGA:
		/*
		m_output_mode.nFPS = 30;
		m_output_mode.nXRes = XN_VGA_X_RES;
		m_output_mode.nYRes = XN_VGA_Y_RES;
		*/
		m_output_mode.setFps(30);
		m_output_mode.setResolution(XN_VGA_X_RES,XN_VGA_Y_RES); //Resolution of VGA : 640*480
		std::cout << "INFO - ColorCamVideoFormat: VGA" << std::endl;
		break;
	default:
		/*
		m_output_mode.nFPS = 30;
		m_output_mode.nXRes = XN_VGA_X_RES;
		m_output_mode.nYRes = XN_VGA_Y_RES;
		*/
		m_output_mode.setFps(30);
		m_output_mode.setResolution(XN_VGA_X_RES,XN_VGA_Y_RES); //Resolution of VGA : 640*480
		std::cout << "INFO - ColorCamVideoFormat: Default" << std::endl;
	}

	//retVal = m_image_generator.SetPixelFormat(XN_PIXEL_FORMAT_RGB24);
	m_output_mode.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
	
	// Kinect
	/*retVal = m_image_generator.SetPixelFormat (XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
	if (retVal != XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not bypass camera internal debayering" << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	*/

	//retVal = m_image_generator.SetMapOutputMode (m_output_mode);
	
	retVal = m_vs_rgb.setVideoMode(m_output_mode);

	//if (retVal !=XN_STATUS_OK)
	if (retVal !=openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot set output mode for color image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}


	//openni2: set ir video mode
	m_output_mode.setPixelFormat(openni::PIXEL_FORMAT_GRAY16);
	retVal = m_vs_ir.setVideoMode(m_output_mode);

	if (retVal !=openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot set output mode for ir image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	m_output_mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
	
	//openni2: set depth video mode
	if (m_ColorCamVideoFormat == SXGA)
	{
		m_output_mode.setResolution(XN_VGA_X_RES,XN_VGA_Y_RES);
	}
	retVal = m_vs_d.setVideoMode(m_output_mode);

	if (retVal !=openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot set output mode for depth image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	


	// Adjust the viewpoint of the range data according to the image data
	/*retVal = m_depth_generator.GetAlternativeViewPointCap().SetViewPoint( m_image_generator );
	if (retVal != XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Error in depth stream to color image registration" << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	*/

	// RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS and Asus
	/*
	retVal = m_depth_generator.SetIntProperty("RegistrationType", 1);
	if (retVal != XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not set registration type";
		return ipa_Utils::RET_FAILED;
	}
	*/

	// Enable the Synchronization of the range data with the image data 
	retVal = m_device.setDepthColorSyncEnabled(true);
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Error in depth stream to color image registration" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS and Asus
	if (m_CalibrationMethod == NATIVE)
	{
		retVal = m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR); //Registration On
		if (retVal != openni::STATUS_OK)
		{
			std::cerr << "ERROR - Kinect::Init:" << std::endl;
			std::cerr << "\t ... Could not set registration type" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
	}
	else if (m_CalibrationMethod == MATLAB_NO_Z)
	{
		/*
		retVal = m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF); //Registration Off
		if (retVal != openni::STATUS_OK)
		{
			std::cerr << "ERROR - Kinect::Init:" << std::endl;
			std::cerr << "\t ... Could not set registration type" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
		*/
	}
	// Set input format to uncompressed 8-bit BAYER
	//retVal = m_image_generator.SetIntProperty ("InputFormat", 6);
	//if (retVal != XN_STATUS_OK)
 //   {
	//	std::cerr << "ERROR - Kinect::Init:" << std::endl;
	//	std::cerr << "\t ... Could not set uncompressed 8-bit BAYER format" << std::endl;
	//	return ipa_Utils::RET_FAILED;
 //   }

	// Bypass camera internal debayering give raw bayer pattern
	// Asus Xtion
	

	//TODO: Find the replacement funtion for shadow value and no sample value (maybe not necessary?)
	/*
	retVal = m_depth_generator.GetIntProperty( "ShadowValue", m_shadowValue );
	if (retVal != XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not read shadow value" << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	  
	retVal = m_depth_generator.GetIntProperty( "NoSampleValue", m_noSampleValue );
	if (retVal != XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not read sample values for invalid disparities" << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	*/

	m_CameraType = ipa_CameraSensors::CAM_KINECT;

	// Set init flag
	m_initialized = true;

	return ipa_Utils::RET_OK;
}


unsigned long Kinect::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}
		
	//XnStatus retVal = XN_STATUS_OK;
	openni::Status retVal = openni::STATUS_OK;

	
	std::cout << "INFO - Kinect::Open():" << std::endl;
	//std::cout << "\t ... Opening camera device" << std::endl;
	std::cout << "\t ... Starting camera streams" << std::endl;
	std::cout << "\t ... This may take some seconds" << std::endl;

	/*
	retVal = m_device.StartGeneratingAll(); 
	if (retVal !=XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Open:" << std::endl;
		std::cerr << "\t ... Cannot start camera device" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	m_depth_generator.GetMetaData(m_depth_md);
	m_image_generator.GetMetaData(m_image_md);

	int width = m_depth_md.XRes();
	int height = m_depth_md.YRes();
	*/

	retVal = m_vs_rgb.start();

	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Open:" << std::endl;
		std::cerr << "\t ... Cannot start color image stream" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	retVal = m_vs_d.start();

	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Open:" << std::endl;
		std::cerr << "\t ... Cannot start depth image stream" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	int width = m_vs_d.getVideoMode().getResolutionX();
	int height = m_vs_d.getVideoMode().getResolutionY();

	m_range_mat.create(height, width, CV_16UC1);

	// Mirror image data
	/*
	xnSetMirror(m_depth_generator, false);
	xnSetMirror(m_image_generator, false);
	*/

	m_vs_rgb.setMirroringEnabled(!m_vs_rgb.getMirroringEnabled());
	m_vs_d.setMirroringEnabled(!m_vs_d.getMirroringEnabled());
	m_vs_ir.setMirroringEnabled(!m_vs_ir.getMirroringEnabled());
	
	std::cout << "*************************************************" << std::endl;
	std::cout << "Kinect::Open: Kinect camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long Kinect::Close()
{
	//m_context.StopGeneratingAll();

	std::cout << "INFO - Kinect: Closing device..." << std::endl;

	m_vs_rgb.stop();
	m_vs_d.stop();
	m_vs_ir.stop();

	m_vs_rgb.destroy();
	m_vs_d.destroy();
	m_vs_ir.destroy();

	m_device.close();
	openni::OpenNI::shutdown();

	m_open = false;
	return RET_OK;
}


unsigned long Kinect::SetProperty(t_cameraProperty* cameraProperty) 
{
	return ipa_Utils::RET_OK;
}


unsigned long Kinect::SetPropertyDefaults() 
{
	return ipa_Utils::RET_OK;
}


unsigned long Kinect::GetProperty(t_cameraProperty* cameraProperty) 
{
	int ret = 0;
	
	//PMDDataDescription dataDescriptor;
	switch (cameraProperty->propertyID)
	{
	case PROP_CAMERA_RESOLUTION:
		if (isOpen())
		{
			// Depth image is upsampled according to the size of the color image
			cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			//cameraProperty->cameraResolution.xResolution = (int) m_image_md.XRes();
			//cameraProperty->cameraResolution.yResolution = (int) m_image_md.YRes();

			cameraProperty->cameraResolution.xResolution = (int) m_vs_rgb.getVideoMode().getResolutionX();
			cameraProperty->cameraResolution.yResolution = (int) m_vs_rgb.getVideoMode().getResolutionY();
		}
		else
		{
			std::cout << "WARNING - Kinect::GetProperty:" << std::endl;
			std::cout << "\t ... Camera not open" << std::endl;
			std::cout << "\t ... Returning default width and height of '640' x '480'" << std::endl;
			cameraProperty->cameraResolution.xResolution = 640;
			cameraProperty->cameraResolution.yResolution = 480;
			cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
		}
		break;

	default: 				
		std::cout << "ERROR - Kinect::GetProperty:" << std::endl;
		std::cout << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
		return ipa_Utils::RET_FAILED;
		break;
	}

	return ipa_Utils::RET_OK;
}


unsigned long Kinect::AcquireImages(cv::Mat* rangeImage, cv::Mat* colorImage, cv::Mat* cartesianImage,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* colorImageData = 0;
	char* cartesianImageData = 0;
	int widthStepRange = -1;
	int widthStepColor = -1;
	int widthStepCartesian = -1;

	//int color_width = m_image_md.XRes();
	//int color_height = m_image_md.YRes();

	int color_width = m_vs_rgb.getVideoMode().getResolutionX();
	int color_height = m_vs_rgb.getVideoMode().getResolutionY();

	if(rangeImage)
	{
		// Depth image is upsampled according to the size of the color image
		rangeImage->create(color_height, color_width, CV_32FC1);
		rangeImageData = rangeImage->ptr<char>(0);
		widthStepRange = rangeImage->step;
	}

	if(colorImage && grayImageType == ipa_CameraSensors::IR)
	{
		colorImage->create(color_height, color_width, CV_16UC1);
		colorImageData = colorImage->ptr<char>(0);
		widthStepColor = colorImage->step;
	}
	else if(colorImage)
	{
		colorImage->create(color_height, color_width, CV_8UC3);
		colorImageData = colorImage->ptr<char>(0);
		widthStepColor = colorImage->step;
	}	

	if(cartesianImage)
	{
		// Depth image is upsampled according to the size of the color image
		cartesianImage->create(color_height, color_width, CV_32FC3);
		cartesianImageData = cartesianImage->ptr<char>(0);
		widthStepCartesian = cartesianImage->step;
	}

	if (!rangeImage && !colorImage && !cartesianImage)
		return RET_OK;

	return AcquireImages(widthStepRange, widthStepColor, widthStepRange, rangeImageData, colorImageData,  cartesianImageData, getLatestFrame, undistort, grayImageType);
}

unsigned long Kinect::AcquireImages(int widthStepRange, int widthStepColor, int widthStepCartesian, char* rangeImageData, char* colorImageData, char* cartesianImageData,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	/*
	int color_width = m_image_md.XRes();
	int color_height = m_image_md.YRes();
	int range_width = m_depth_md.XRes();
	int range_height = m_depth_md.YRes();
	*/

	int color_width = m_vs_rgb.getVideoMode().getResolutionX();
	int color_height = m_vs_rgb.getVideoMode().getResolutionY();
	
	int range_width = m_vs_d.getVideoMode().getResolutionX();
	int range_height = m_vs_d.getVideoMode().getResolutionY();

	/*
	m_depth_generator.WaitAndUpdateData();
	m_depth_generator.GetMetaData(m_depth_md);
	m_image_generator.WaitAndUpdateData();
	m_image_generator.GetMetaData(m_image_md);
	*/

	// Wait for all streams
	int changedIndex;

	// declare temp streams
	openni::VideoStream* tempStream_rgb;
	openni::VideoStream* tempStream_d;
	openni::VideoStream* tempStream_ir;
	openni::Status retVal;

	//get depth frame
	tempStream_d = &m_vs_d;
	retVal = openni::OpenNI::waitForAnyStream(&tempStream_d, 1, &changedIndex, 2000); //2000ms
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::AcquireImages" << std::endl;
		std::cerr << "\t ... Wait limit for depth stream exceeded" << std::endl;
		return ipa_Utils::RET_FAILED;;
	}
	m_vs_d.readFrame(&m_vfr_d); 
	
	//Debug: show the mittel pixel value
	/*
	openni::DepthPixel* pDepth = (openni::DepthPixel*)m_vfr_d.getData();
	int middleIndex = (m_vfr_d.getHeight()+1)*m_vfr_d.getWidth()/2;
	printf("[%08llu] %8d\n", (long long)m_vfr_d.getTimestamp(), pDepth[middleIndex]);
	*/

	//get color frame
	tempStream_rgb = &m_vs_rgb;
	retVal = openni::OpenNI::waitForAnyStream(&tempStream_rgb, 1, &changedIndex, 2000); //2000ms
	if (retVal != openni::STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::AcquireImages" << std::endl;
		std::cerr << "\t ... Wait limit for color stream exceeded" << std::endl;
		return ipa_Utils::RET_FAILED;;
	}
	m_vs_rgb.readFrame(&m_vfr_rgb); 

	//get ir frame
	if(grayImageType == IR)
	{
		m_vs_rgb.stop();
		m_vs_d.stop();

		retVal = m_vs_ir.start();
		if (retVal != openni::STATUS_OK)
		{
			std::cerr << "ERROR - Kinect::AcquireImages" << std::endl;
			std::cerr << "\t [FAILED] Starting IR stream" << std::endl;
			return ipa_Utils::RET_FAILED;;
		}

		tempStream_ir = &m_vs_ir;
		retVal = openni::OpenNI::waitForAnyStream(&tempStream_ir, 1, &changedIndex, 2000); //2000ms
		if (retVal != openni::STATUS_OK)
		{
			std::cerr << "ERROR - Kinect::AcquireImages" << std::endl;
			std::cerr << "\t ... Wait limit for IR stream exceeded" << std::endl;
			return ipa_Utils::RET_FAILED;;
		}
		
		m_vs_ir.readFrame(&m_vfr_ir); 

		m_vfr_ir.getData();

		m_vs_ir.stop();
		m_vs_rgb.start();
		m_vs_d.start();

	}
	//// TODO: Use real calibration data
	//float scale = color_width / (float)XN_SXGA_X_RES;
	//// focal length for regular camera producing color images in native SXGA mode
	//float rgb_focal_length_SXGA_ = 1050; 
	//// depth values are registered to color camera
	//float depth_registered_focal_length =  rgb_focal_length_SXGA_ * scale;
	//float constant = 0.001 / depth_registered_focal_length;
	//float cx = (color_width >> 1 ) - 0.5f;
	//float cy = (color_height >> 1) - 0.5f;

	if (m_intrinsicMatrix.empty())
	{
		std::cout << "ERROR - Kinect::AcquireImages:" << std::endl;
		std::cout << "\t ... Intrinsic matrix not initialized\n";
		std::cout << "\t ... Aborting\n";
		return ipa_Utils::RET_FAILED;
	}

	if (m_extrinsicMatrix.empty())
	{
		std::cout << "ERROR - Kinect::AcquireImages:" << std::endl;
		std::cout << "\t ... Extrinsic matrix not initialized\n";
		std::cout << "\t ... Aborting\n";
		return ipa_Utils::RET_FAILED;
	}


	double fx, fy, cx, cy;
	fx = m_intrinsicMatrix.at<double>(0, 0);
	fy = m_intrinsicMatrix.at<double>(1, 1);
	cx = m_intrinsicMatrix.at<double>(0, 2);
	cy = m_intrinsicMatrix.at<double>(1, 2);
	float constant_x = 0.001 / fx;
	float constant_y = 0.001 / fy;

	if(rangeImageData || cartesianImageData)
	{
		float* p_xyz = 0;
		unsigned short* p_us_dist = 0;
		float* p_f_dist = 0;
		int colTimes3;

		bool createRangeImage = (rangeImageData != 0);
		bool createXYZImage = (cartesianImageData != 0);

		// Get z values
		char* range_mat_ptr = m_range_mat.ptr<char>(0);
		//memcpy( range_mat_ptr, m_depth_md.Data(), range_height * range_width * sizeof(XnDepthPixel) );
		memcpy( range_mat_ptr, m_vfr_d.getData(), range_height * range_width * sizeof(openni::DepthPixel) ); 

		// Resize if necessary
		cv::Mat resized_range_mat;
		if (m_ColorCamVideoFormat == SXGA)
		{
			
			range_width = XN_SXGA_X_RES;
			range_height = XN_SXGA_Y_RES;

			resized_range_mat.create(XN_SXGA_Y_RES, XN_SXGA_X_RES, m_range_mat.type());
			resized_range_mat.setTo(cv::Scalar(m_badDepth));
			int y_idx = 0.5*(XN_SXGA_Y_RES-(2*XN_VGA_Y_RES)-1);
			int x_idx = 0.5*(XN_SXGA_X_RES-2*XN_VGA_X_RES-1);
			cv::Mat sub_mat = resized_range_mat.rowRange(y_idx, y_idx + 2*XN_VGA_Y_RES);
			cv::resize(m_range_mat, sub_mat, cv::Size(), 2, 2, CV_INTER_NN);
		}
		else
			resized_range_mat = m_range_mat;
		
		// Convert zuv values to float xyz
		cv::Mat transformedXYZ = cv::Mat::zeros(range_height, range_width, CV_32FC3);
		if (createXYZImage)
		{
			int xzy_step = range_width * sizeof(float) * 3;
			int d_step = range_width * sizeof(unsigned short);
			
			m_XYZ.create(3, 1, CV_64FC1);

			double* d_ptr = 0;
			double x,y,z;
			int u = 0, v = 0;

			//For fast calculation: avoid using matrix multiplication
			d_ptr = m_extrinsicMatrix.ptr<double>(0);
			
			double er11 = d_ptr[0], er12 = d_ptr[1], er13 = d_ptr[2], et1 = d_ptr[3],
				   er21 = d_ptr[4], er22 = d_ptr[5], er23 = d_ptr[6], et2 = d_ptr[7],
				   er31 = d_ptr[8], er32 = d_ptr[9], er33 = d_ptr[10], et3 = d_ptr[11];

			for(unsigned int row=0; row<(unsigned int)range_height; row++)
			{
				p_xyz = (float*) (cartesianImageData + row * xzy_step);
				p_us_dist = resized_range_mat.ptr<unsigned short>(row);

				for (unsigned int col = 0; col < (unsigned int)range_width; col++)
				{
					colTimes3 = 3*col;
					// Check for invalid measurements
					if( p_us_dist[col] == m_badDepth ) // not valid
					{
						p_xyz[colTimes3] = 0;
						p_xyz[colTimes3 + 1] = 0;
						p_xyz[colTimes3 + 2] = 0;
					}
					else
					{
						p_xyz[colTimes3] = (col - cx) * p_us_dist[col] * constant_x;
						p_xyz[colTimes3 + 1] = (row - cy) * p_us_dist[col] * constant_y;
						p_xyz[colTimes3 + 2] = p_us_dist[col] * 0.001;


						// Calculate the registrated coordinates if the registration is off
						if (m_CalibrationMethod == MATLAB_NO_Z)
						{	
							x = p_xyz[colTimes3];
							y = p_xyz[colTimes3 + 1];
							z = p_xyz[colTimes3 + 2];

							x *= 1000.0;
							y *= 1000.0;
							z *= 1000.0;

							d_ptr =  m_XYZ.ptr<double>(0);

							d_ptr[0] =  er11 * x + er12 * y + er13 * z + et1;
							d_ptr[1] =  er21 * x + er22 * y + er23 * z + et2;
							d_ptr[2] =  er31 * x + er32 * y + er33 * z + et3 + m_dZ;

							u = cvRound(fx * d_ptr[0] / d_ptr[2] + cx);
							v = cvRound(fy * d_ptr[1] / d_ptr[2] + cy);

							d_ptr = m_XYZ.ptr<double>(0);

							if (u < range_width && v < range_height && u >= 0 && v >=0)
							{
								transformedXYZ.at<cv::Vec3f>(v, u) =  cv::Vec3f(d_ptr[0]*0.001, d_ptr[1]*0.001,d_ptr[2]*0.001);		
							}
						}
						
						//if (p_xyz[colTimes3 + 2] < 0)
						//{
						//	std::cout << "<0: " << row << " " << col << "\n";
						//}

						//XnPoint3D proj, real;
						//proj.X = col;
						//proj.Y = row;
						//proj.Z = p_dist[col];
						//m_depth_generator.ConvertProjectiveToRealWorld(1, &proj, &real);
						//p_xyz[colTimes3] = real.X* 0.001f;
						//p_xyz[colTimes3 + 1] = real.Y* 0.001f;
						//p_xyz[colTimes3 + 2] = real.Z* 0.001f;
					}
				}
			}
			if (m_CalibrationMethod == MATLAB_NO_Z)
			{
				memcpy (( unsigned char*) cartesianImageData, transformedXYZ.ptr(0), range_width * sizeof(float) * 3 * range_height);
			}
		}

		// Convert z values to float z
		if (createRangeImage)
		{
			// Get range values
			unsigned short us_val = 0;
			int d_step = range_width * sizeof(float);

			for(unsigned int row=0; row<(unsigned int)range_height; row++)
			{
				p_f_dist = (float*)(rangeImageData + row * d_step);
				p_us_dist = resized_range_mat.ptr<unsigned short>(row);

				for (unsigned int col=0; col<(unsigned int)range_width; col++)
				{
					us_val = p_us_dist[col];
					// Convert to float to stay consistent with other range cameras
					//if (us_val != m_noSampleValue && 
					//	us_val != m_shadowValue &&
					//	us_val != 0)
					if (us_val != 0) // TODO: find out wether it is necessary to get the shadow and no sample value
						p_f_dist[col] = 0.001 * (float)us_val;
					else
						p_f_dist[col] = (float)m_badDepth;
				}	
			}
		}
	}

	if (grayImageType == ipa_CameraSensors::IR && colorImageData)
	{
		// TODO : handle the IR image here!!!
		int ir_width = m_vfr_ir.getVideoMode().getResolutionX();
		int ir_height = m_vfr_ir.getVideoMode().getResolutionY();
		size_t pix_size = sizeof(openni::Grayscale16Pixel);
		//memcpy (( unsigned char*) colorImageData, m_vfr_ir.getData(), m_vfr_ir.getDataSize());
		memcpy (( unsigned char*) colorImageData, m_vfr_ir.getData(), pix_size*ir_width*ir_height);
	}
	// Read out the color image
	// Check if new data is available
	else if (colorImageData)
	{
		openni::PixelFormat rgb_pf = m_vs_rgb.getVideoMode().getPixelFormat();
		//if (XnPixelFormat::XN_PIXEL_FORMAT_GRAYSCALE_8_BIT == m_image_md.PixelFormat())
		if (openni::PIXEL_FORMAT_GRAY8 == rgb_pf)
			FillRGBBayer(color_width, color_height, ( unsigned char*) colorImageData);
		//else if (XnPixelFormat::XN_PIXEL_FORMAT_YUV422 == m_image_md.PixelFormat())
		else if (openni::PIXEL_FORMAT_YUV422 == rgb_pf)
			FillRGBYUV422(color_width, color_height, ( unsigned char*) colorImageData);
		//else if (XnPixelFormat::XN_PIXEL_FORMAT_RGB24 == m_image_md.PixelFormat())
		else if (openni::PIXEL_FORMAT_RGB888 == rgb_pf)
			//memcpy (( unsigned char*) colorImageData, m_image_md.Data(), m_image_md.DataSize());
			//memcpy (( unsigned char*) colorImageData, m_vfr_rgb.getData(), m_vfr_rgb.getDataSize()); 
			memcpy (( unsigned char*) colorImageData, m_vfr_rgb.getData(), m_vfr_rgb.getDataSize()); 
		else
		{
			std::cout << "ERROR - Kinect::AcquireImages:" << std::endl;
			std::cout << "\t ... Unsupported pixel format";
			return ipa_Utils::RET_FAILED;
		}

		// Switch red and blue image channels
		unsigned char temp_val = 0;
		unsigned char* p_dest = 0;
		unsigned int colTimes3 = 0;
		int colInverse = 0;
		for(unsigned int row=0; row<(unsigned int)color_height; row++)
		{
			p_dest = ((unsigned char*)colorImageData) + row * color_width * 3;
			for (unsigned int col=0; col<(unsigned int)color_width; col++)
			{
				colTimes3 = col*3;

				// Switch red and blue image channels
				temp_val = p_dest[colTimes3];
				p_dest[colTimes3] = p_dest[colTimes3 + 2];
				p_dest[colTimes3 + 2] = temp_val;
			}	
		}
	}

	return  RET_OK;
}

unsigned long Kinect::FillRGBYUV422(unsigned width, unsigned height, unsigned char* rgb_buffer)
{
	// 0  1   2  3
	// u  y1  v  y2

	//if (m_image_md.XRes() != width && m_image_md.YRes() != height)
	if (m_vs_rgb.getVideoMode().getResolutionX() != width && m_vs_rgb.getVideoMode().getResolutionY() != height)
	{
		//if (width > m_image_md.XRes () || height > m_image_md.YRes ())
		if (width > m_vs_rgb.getVideoMode().getResolutionX() || height > m_vs_rgb.getVideoMode().getResolutionY())
			std::cerr << "ERROR - Kinect::FillRGBYUV422" << std::endl;
			std::cerr << "\t ... Upsampling not supported" << std::endl;
			return RET_FAILED;

		//if ( m_image_md.XRes () % width != 0 || m_image_md.YRes () % height != 0
		//	|| (m_image_md.XRes () / width) & 0x01 || (m_image_md.YRes () / height & 0x01) )
		if ( m_vs_rgb.getVideoMode().getResolutionX() % width != 0 ||m_vs_rgb.getVideoMode().getResolutionY() % height != 0
			|| (m_vs_rgb.getVideoMode().getResolutionX() / width) & 0x01 || (m_vs_rgb.getVideoMode().getResolutionY() / height & 0x01) )
			std::cerr << "ERROR - Kinect::FillRGBYUV422" << std::endl;
			std::cerr << "\t ... Downsampling only possible for power of two scale in both dimensions" << std::endl;
			return RET_FAILED;
	}

	//register const XnUInt8* yuv_buffer = m_image_md.Data();
	
	register const uint8_t* yuv_buffer = (const uint8_t*) m_vfr_rgb.getData(); 

	unsigned int rgb_line_step = width * 3 * 2;
	unsigned rgb_line_skip = 0;
	if (rgb_line_step != 0)
		rgb_line_skip = rgb_line_step - width * 3;

	//if (m_image_md.XRes() == width && m_image_md.YRes() == height)
	if (m_vs_rgb.getVideoMode().getResolutionX() == width && m_vs_rgb.getVideoMode().getResolutionY() == height)
	{
		for( register unsigned yIdx = 0; yIdx < height; ++yIdx, rgb_buffer += rgb_line_skip )
		{
			for( register unsigned xIdx = 0; xIdx < width; xIdx += 2, rgb_buffer += 6, yuv_buffer += 4 )
			{
				int v = yuv_buffer[2] - 128;
				int u = yuv_buffer[0] - 128;

				rgb_buffer[0] =  IPA_CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
				rgb_buffer[1] =  IPA_CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
				rgb_buffer[2] =  IPA_CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));

				rgb_buffer[3] =  IPA_CLIP_CHAR (yuv_buffer[3] + ((v * 18678 + 8192 ) >> 14));
				rgb_buffer[4] =  IPA_CLIP_CHAR (yuv_buffer[3] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
				rgb_buffer[5] =  IPA_CLIP_CHAR (yuv_buffer[3] + ((u * 33292 + 8192 ) >> 14));
			}
		}
	}
	else
	{
		//register unsigned yuv_step = m_image_md.XRes() / width;
		register unsigned yuv_step = m_vs_rgb.getVideoMode().getResolutionX() / width;
		register unsigned yuv_x_step = yuv_step << 1;
		//register unsigned yuv_skip = (m_image_md.YRes() / height - 1) * ( m_image_md.XRes() << 1 );
		register unsigned yuv_skip = (m_vs_rgb.getVideoMode().getResolutionY() / height - 1) * (m_vs_rgb.getVideoMode().getResolutionX() << 1 );

		//for( register unsigned yIdx = 0; yIdx < m_image_md.YRes(); yIdx += yuv_step, yuv_buffer += yuv_skip, rgb_buffer += rgb_line_skip )
		for( register unsigned yIdx = 0; yIdx <m_vs_rgb.getVideoMode().getResolutionY(); yIdx += yuv_step, yuv_buffer += yuv_skip, rgb_buffer += rgb_line_skip )
		{
			//for( register unsigned xIdx = 0; xIdx < m_image_md.XRes(); xIdx += yuv_step, rgb_buffer += 3, yuv_buffer += yuv_x_step )
			for( register unsigned xIdx = 0; xIdx < m_vs_rgb.getVideoMode().getResolutionX(); xIdx += yuv_step, rgb_buffer += 3, yuv_buffer += yuv_x_step )
			{
				int v = yuv_buffer[2] - 128;
				int u = yuv_buffer[0] - 128;

				rgb_buffer[0] =  IPA_CLIP_CHAR (yuv_buffer[1] + ((v * 18678 + 8192 ) >> 14));
				rgb_buffer[1] =  IPA_CLIP_CHAR (yuv_buffer[1] + ((v * -9519 - u * 6472 + 8192 ) >> 14));
				rgb_buffer[2] =  IPA_CLIP_CHAR (yuv_buffer[1] + ((u * 33292 + 8192 ) >> 14));
			}
		}
	}
	return  RET_OK;
}

unsigned long Kinect::FillRGBBayer(unsigned width, unsigned height, unsigned char* rgb_buffer)
{
	//if (width != m_image_md.XRes () || height != m_image_md.YRes ())
	if (width != m_vs_rgb.getVideoMode().getResolutionX() || height != m_vs_rgb.getVideoMode().getResolutionY())
		// TODO: Throw exception
		return RET_FAILED;
	
	unsigned int rgb_line_step = width * 3;

	// padding skip for destination image
	unsigned rgb_line_skip = rgb_line_step - width * 3;

	//register const XnUInt8 *bayer_pixel = m_image_md.Data ();
	register const uint8_t *bayer_pixel = (const uint8_t *) m_vfr_rgb.getData(); //TODO: Type Error?
	
	register unsigned yIdx, xIdx;

	//int bayer_line_step = m_image_md.XRes ();
	//int bayer_line_step2 = m_image_md.XRes () << 1;
	int bayer_line_step = m_vs_rgb.getVideoMode().getResolutionX();
	int bayer_line_step2 = m_vs_rgb.getVideoMode().getResolutionX() << 1;

	//int debayering_method = 0; // Bilinear
	//int debayering_method = 1; // Edge aware
	int debayering_method = 2; // EdgeAwareWeighted

	if (debayering_method == 0) // Bilinar
	{
		// first two pixel values for first two lines
		// Bayer         0 1 2
		//         0     G r g
		// line_step     b g b
		// line_step2    g r g

		rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
		rgb_buffer[1] = bayer_pixel[0]; // green pixel
		rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

		// Bayer         0 1 2
		//         0     g R g
		// line_step     b g b
		// line_step2    g r g
		//rgb_pixel[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

		// BGBG line
		// Bayer         0 1 2
		//         0     g r g
		// line_step     B g b
		// line_step2    g r g
		rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
		rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
		//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

		// pixel (1, 1)  0 1 2
		//         0     g r g
		// line_step     b G b
		// line_step2    g r g
		//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

		rgb_buffer += 6;
		bayer_pixel += 2;
		// rest of the first two lines

		for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
		{
			// GRGR line
			// Bayer        -1 0 1 2
			//           0   r G r g
			//   line_step   g b g b
			// line_step2    r g r g
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

			// Bayer        -1 0 1 2
			//          0    r g R g
			//  line_step    g b g b
			// line_step2    r g r g
			rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

			// BGBG line
			// Bayer         -1 0 1 2
			//         0      r g r g
			// line_step      g B g b
			// line_step2     r g r g
			rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
			rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

			// Bayer         -1 0 1 2
			//         0      r g r g
			// line_step      g b G b
			// line_step2     r g r g
			rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			//rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
		}

		// last two pixel values for first two lines
		// GRGR line
		// Bayer        -1 0 1
		//           0   r G r
		//   line_step   g b g
		// line_step2    r g r
		rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
		rgb_buffer[1] = bayer_pixel[0];
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

		// Bayer        -1 0 1
		//          0    r g R
		//  line_step    g b g
		// line_step2    r g r
		rgb_buffer[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
		//rgb_pixel[5] = bayer_pixel[line_step];

		// BGBG line
		// Bayer        -1 0 1
		//          0    r g r
		//  line_step    g B g
		// line_step2    r g r
		rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
		rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
		//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

		// Bayer         -1 0 1
		//         0      r g r
		// line_step      g b G
		// line_step2     r g r
		rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

		bayer_pixel += bayer_line_step + 2;
		rgb_buffer += rgb_line_step + 6 + rgb_line_skip;

		// main processing

		for (yIdx = 2; yIdx < height - 2; yIdx += 2)
		{
			// first two pixel values
			// Bayer         0 1 2
			//        -1     b g b
			//         0     G r g
			// line_step     b g b
			// line_step2    g r g

			rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
			rgb_buffer[1] = bayer_pixel[0]; // green pixel
			rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

			// Bayer         0 1 2
			//        -1     b g b
			//         0     g R g
			// line_step     b g b
			// line_step2    g r g
			//rgb_pixel[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
			rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

			// BGBG line
			// Bayer         0 1 2
			//         0     g r g
			// line_step     B g b
			// line_step2    g r g
			rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

			// pixel (1, 1)  0 1 2
			//         0     g r g
			// line_step     b G b
			// line_step2    g r g
			//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

			rgb_buffer += 6;
			bayer_pixel += 2;
			// continue with rest of the line
			for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
			{
				// GRGR line
				// Bayer        -1 0 1 2
				//          -1   g b g b
				//           0   r G r g
				//   line_step   g b g b
				// line_step2    r g r g
				rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
				rgb_buffer[1] = bayer_pixel[0];
				rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

				// Bayer        -1 0 1 2
				//          -1   g b g b
				//          0    r g R g
				//  line_step    g b g b
				// line_step2    r g r g
				rgb_buffer[3] = bayer_pixel[1];
				rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
				rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

				// BGBG line
				// Bayer         -1 0 1 2
				//         -1     g b g b
				//          0     r g r g
				// line_step      g B g b
				// line_step2     r g r g
				rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
				rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
				rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

				// Bayer         -1 0 1 2
				//         -1     g b g b
				//          0     r g r g
				// line_step      g b G b
				// line_step2     r g r g
				rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
				rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
				rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
			}

			// last two pixels of the line
			// last two pixel values for first two lines
			// GRGR line
			// Bayer        -1 0 1
			//           0   r G r
			//   line_step   g b g
			// line_step2    r g r
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

			// Bayer        -1 0 1
			//          0    r g R
			//  line_step    g b g
			// line_step2    r g r
			rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
			//rgb_pixel[5] = bayer_pixel[line_step];

			// BGBG line
			// Bayer        -1 0 1
			//          0    r g r
			//  line_step    g B g
			// line_step2    r g r
			rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
			rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

			// Bayer         -1 0 1
			//         0      r g r
			// line_step      g b G
			// line_step2     r g r
			rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

			bayer_pixel += bayer_line_step + 2;
			rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
		}

		//last two lines
		// Bayer         0 1 2
		//        -1     b g b
		//         0     G r g
		// line_step     b g b

		rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
		rgb_buffer[1] = bayer_pixel[0]; // green pixel
		rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

		// Bayer         0 1 2
		//        -1     b g b
		//         0     g R g
		// line_step     b g b
		//rgb_pixel[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
		rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

		// BGBG line
		// Bayer         0 1 2
		//        -1     b g b
		//         0     g r g
		// line_step     B g b
		//rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
		rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

		// Bayer         0 1 2
		//        -1     b g b
		//         0     g r g
		// line_step     b G b
		//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

		rgb_buffer += 6;
		bayer_pixel += 2;
		// rest of the last two lines
		for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
		{
			// GRGR line
			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r G r g
			// line_step    g b g b
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g R g
			// line_step    g b g b
			rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
			rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

			// BGBG line
			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g r g
			// line_step    g B g b
			rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
			rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g r g
			// line_step    g b G b
			//rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
		}

		// last two pixel values for first two lines
		// GRGR line
		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r G r
		// line_step    g b g
		rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
		rgb_buffer[1] = bayer_pixel[0];
		rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g R
		// line_step    g b g
		rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
		//rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

		// BGBG line
		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g r
		// line_step    g B g
		//rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
		rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g r
		// line_step    g b G
		//rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
	}
	else if (debayering_method == 1) // Edge aware
	{
		int dh, dv;

		// first two pixel values for first two lines
		// Bayer         0 1 2
		//         0     G r g
		// line_step     b g b
		// line_step2    g r g

		rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
		rgb_buffer[1] = bayer_pixel[0]; // green pixel
		rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

		// Bayer         0 1 2
		//         0     g R g
		// line_step     b g b
		// line_step2    g r g
		//rgb_pixel[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

		// BGBG line
		// Bayer         0 1 2
		//         0     g r g
		// line_step     B g b
		// line_step2    g r g
		rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
		rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
		//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

		// pixel (1, 1)  0 1 2
		//         0     g r g
		// line_step     b G b
		// line_step2    g r g
		//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

		rgb_buffer += 6;
		bayer_pixel += 2;
		// rest of the first two lines
		for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
		{
			// GRGR line
			// Bayer        -1 0 1 2
			//           0   r G r g
			//   line_step   g b g b
			// line_step2    r g r g
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

			// Bayer        -1 0 1 2
			//          0    r g R g
			//  line_step    g b g b
			// line_step2    r g r g
			rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

			// BGBG line
			// Bayer         -1 0 1 2
			//         0      r g r g
			// line_step      g B g b
			// line_step2     r g r g
			rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
			rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

			// Bayer         -1 0 1 2
			//         0      r g r g
			// line_step      g b G b
			// line_step2     r g r g
			rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			//rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
		}

		// last two pixel values for first two lines
		// GRGR line
		// Bayer        -1 0 1
		//           0   r G r
		//   line_step   g b g
		// line_step2    r g r
		rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
		rgb_buffer[1] = bayer_pixel[0];
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

		// Bayer        -1 0 1
		//          0    r g R
		//  line_step    g b g
		// line_step2    r g r
		rgb_buffer[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
		//rgb_pixel[5] = bayer_pixel[line_step];

		// BGBG line
		// Bayer        -1 0 1
		//          0    r g r
		//  line_step    g B g
		// line_step2    r g r
		rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
		rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
		//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

		// Bayer         -1 0 1
		//         0      r g r
		// line_step      g b G
		// line_step2     r g r
		rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

		bayer_pixel += bayer_line_step + 2;
		rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
		// main processing
		for (yIdx = 2; yIdx < height - 2; yIdx += 2)
		{
			// first two pixel values
			// Bayer         0 1 2
			//        -1     b g b
			//         0     G r g
			// line_step     b g b
			// line_step2    g r g

			rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
			rgb_buffer[1] = bayer_pixel[0]; // green pixel
			rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

			// Bayer         0 1 2
			//        -1     b g b
			//         0     g R g
			// line_step     b g b
			// line_step2    g r g
			//rgb_pixel[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
			rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

			// BGBG line
			// Bayer         0 1 2
			//         0     g r g
			// line_step     B g b
			// line_step2    g r g
			rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

			// pixel (1, 1)  0 1 2
			//         0     g r g
			// line_step     b G b
			// line_step2    g r g
			//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

			rgb_buffer += 6;
			bayer_pixel += 2;
			// continue with rest of the line
			for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
			{
				// GRGR line
				// Bayer        -1 0 1 2
				//          -1   g b g b
				//           0   r G r g
				//   line_step   g b g b
				// line_step2    r g r g
				rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
				rgb_buffer[1] = bayer_pixel[0];
				rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

				// Bayer        -1 0 1 2
				//          -1   g b g b
				//          0    r g R g
				//  line_step    g b g b
				// line_step2    r g r g

				dh = abs (bayer_pixel[0] - bayer_pixel[2]);
				dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

				if (dh > dv)
					rgb_buffer[4] = AVG (bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1]);
				else if (dv > dh)
					rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[2]);
				else
					rgb_buffer[4] = AVG4 (bayer_pixel[-bayer_line_step + 1], bayer_pixel[bayer_line_step + 1], bayer_pixel[0], bayer_pixel[2]);

				rgb_buffer[3] = bayer_pixel[1];
				rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

				// BGBG line
				// Bayer         -1 0 1 2
				//         -1     g b g b
				//          0     r g r g
				// line_step      g B g b
				// line_step2     r g r g
				rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
				rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

				dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
				dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

				if (dv > dh)
					rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
				else if (dh > dv)
					rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step2]);
				else
					rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);

				// Bayer         -1 0 1 2
				//         -1     g b g b
				//          0     r g r g
				// line_step      g b G b
				// line_step2     r g r g
				rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
				rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
				rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
			}

			// last two pixels of the line
			// last two pixel values for first two lines
			// GRGR line
			// Bayer        -1 0 1
			//           0   r G r
			//   line_step   g b g
			// line_step2    r g r
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

			// Bayer        -1 0 1
			//          0    r g R
			//  line_step    g b g
			// line_step2    r g r
			rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
			//rgb_pixel[5] = bayer_pixel[line_step];

			// BGBG line
			// Bayer        -1 0 1
			//          0    r g r
			//  line_step    g B g
			// line_step2    r g r
			rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
			rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

			// Bayer         -1 0 1
			//         0      r g r
			// line_step      g b G
			// line_step2     r g r
			rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

			bayer_pixel += bayer_line_step + 2;
			rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
		}

		//last two lines
		// Bayer         0 1 2
		//        -1     b g b
		//         0     G r g
		// line_step     b g b

		rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
		rgb_buffer[1] = bayer_pixel[0]; // green pixel
		rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

		// Bayer         0 1 2
		//        -1     b g b
		//         0     g R g
		// line_step     b g b
		//rgb_pixel[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
		rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

		// BGBG line
		// Bayer         0 1 2
		//        -1     b g b
		//         0     g r g
		// line_step     B g b
		//rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
		rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

		// Bayer         0 1 2
		//        -1     b g b
		//         0     g r g
		// line_step     b G b
		//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

		rgb_buffer += 6;
		bayer_pixel += 2;
		// rest of the last two lines
		for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
		{
			// GRGR line
			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r G r g
			// line_step    g b g b
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g R g
			// line_step    g b g b
			rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
			rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

			// BGBG line
			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g r g
			// line_step    g B g b
			rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
			rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g r g
			// line_step    g b G b
			//rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
		}

		// last two pixel values for first two lines
		// GRGR line
		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r G r
		// line_step    g b g
		rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
		rgb_buffer[1] = bayer_pixel[0];
		rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g R
		// line_step    g b g
		rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
		//rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

		// BGBG line
		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g r
		// line_step    g B g
		//rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
		rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g r
		// line_step    g b G
		//rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
	}
	else if (debayering_method == 2) // EdgeAwareWeighted
	{
		int dh, dv;

		// first two pixel values for first two lines
		// Bayer         0 1 2
		//         0     G r g
		// line_step     b g b
		// line_step2    g r g

		rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
		rgb_buffer[1] = bayer_pixel[0]; // green pixel
		rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

		// Bayer         0 1 2
		//         0     g R g
		// line_step     b g b
		// line_step2    g r g
		//rgb_pixel[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

		// BGBG line
		// Bayer         0 1 2
		//         0     g r g
		// line_step     B g b
		// line_step2    g r g
		rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
		rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
		//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

		// pixel (1, 1)  0 1 2
		//         0     g r g
		// line_step     b G b
		// line_step2    g r g
		//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );

		rgb_buffer += 6;
		bayer_pixel += 2;
		// rest of the first two lines
		for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
		{
			// GRGR line
			// Bayer        -1 0 1 2
			//           0   r G r g
			//   line_step   g b g b
			// line_step2    r g r g
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[2] = bayer_pixel[bayer_line_step + 1];

			// Bayer        -1 0 1 2
			//          0    r g R g
			//  line_step    g b g b
			// line_step2    r g r g
			rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 5] = rgb_buffer[5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

			// BGBG line
			// Bayer         -1 0 1 2
			//         0      r g r g
			// line_step      g B g b
			// line_step2     r g r g
			rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
			rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

			// Bayer         -1 0 1 2
			//         0      r g r g
			// line_step      g b G b
			// line_step2     r g r g
			rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			//rgb_pixel[rgb_line_step + 5] = AVG( bayer_pixel[line_step] , bayer_pixel[line_step+2] );
		}

		// last two pixel values for first two lines
		// GRGR line
		// Bayer        -1 0 1
		//           0   r G r
		//   line_step   g b g
		// line_step2    r g r
		rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
		rgb_buffer[1] = bayer_pixel[0];
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

		// Bayer        -1 0 1
		//          0    r g R
		//  line_step    g b g
		// line_step2    r g r
		rgb_buffer[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
		//rgb_pixel[5] = bayer_pixel[line_step];

		// BGBG line
		// Bayer        -1 0 1
		//          0    r g r
		//  line_step    g B g
		// line_step2    r g r
		rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
		rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
		//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

		// Bayer         -1 0 1
		//         0      r g r
		// line_step      g b G
		// line_step2     r g r
		rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

		bayer_pixel += bayer_line_step + 2;
		rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
		// main processing
		for (yIdx = 2; yIdx < height - 2; yIdx += 2)
		{
			// first two pixel values
			// Bayer         0 1 2
			//        -1     b g b
			//         0     G r g
			// line_step     b g b
			// line_step2    g r g

			rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
			rgb_buffer[1] = bayer_pixel[0]; // green pixel
			rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]); // blue;

			// Bayer         0 1 2
			//        -1     b g b
			//         0     g R g
			// line_step     b g b
			// line_step2    g r g
			//rgb_pixel[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
			rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

			// BGBG line
			// Bayer         0 1 2
			//         0     g r g
			// line_step     B g b
			// line_step2    g r g
			rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[bayer_line_step2]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

			// pixel (1, 1)  0 1 2
			//         0     g r g
			// line_step     b G b
			// line_step2    g r g
			//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

			rgb_buffer += 6;
			bayer_pixel += 2;
			// continue with rest of the line
			for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
			{
				// GRGR line
				// Bayer        -1 0 1 2
				//          -1   g b g b
				//           0   r G r g
				//   line_step   g b g b
				// line_step2    r g r g
				rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
				rgb_buffer[1] = bayer_pixel[0];
				rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

				// Bayer        -1 0 1 2
				//          -1   g b g b
				//          0    r g R g
				//  line_step    g b g b
				// line_step2    r g r g

				dh = abs (bayer_pixel[0] - bayer_pixel[2]);
				dv = abs (bayer_pixel[-bayer_line_step + 1] - bayer_pixel[bayer_line_step + 1]);

				if (dv == 0 && dh == 0)
					rgb_buffer[4] = AVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2]);
				else
					rgb_buffer[4] = WAVG4 (bayer_pixel[1 - bayer_line_step], bayer_pixel[1 + bayer_line_step], bayer_pixel[0], bayer_pixel[2], dh, dv);
				rgb_buffer[3] = bayer_pixel[1];
				rgb_buffer[5] = AVG4 (bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step], bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

				// BGBG line
				// Bayer         -1 0 1 2
				//         -1     g b g b
				//          0     r g r g
				// line_step      g B g b
				// line_step2     r g r g
				rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
				rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

				dv = abs (bayer_pixel[0] - bayer_pixel[bayer_line_step2]);
				dh = abs (bayer_pixel[bayer_line_step - 1] - bayer_pixel[bayer_line_step + 1]);

				if (dv == 0 && dh == 0)
					rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
				else
					rgb_buffer[rgb_line_step + 1] = WAVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1], dh, dv);

				// Bayer         -1 0 1 2
				//         -1     g b g b
				//          0     r g r g
				// line_step      g b G b
				// line_step2     r g r g
				rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
				rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
				rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
			}

			// last two pixels of the line
			// last two pixel values for first two lines
			// GRGR line
			// Bayer        -1 0 1
			//           0   r G r
			//   line_step   g b g
			// line_step2    r g r
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = rgb_buffer[5] = rgb_buffer[2] = bayer_pixel[bayer_line_step];

			// Bayer        -1 0 1
			//          0    r g R
			//  line_step    g b g
			// line_step2    r g r
			rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
			//rgb_pixel[5] = bayer_pixel[line_step];

			// BGBG line
			// Bayer        -1 0 1
			//          0    r g r
			//  line_step    g B g
			// line_step2    r g r
			rgb_buffer[rgb_line_step ] = AVG4 (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1], bayer_pixel[-1], bayer_pixel[bayer_line_step2 - 1]);
			rgb_buffer[rgb_line_step + 1] = AVG4 (bayer_pixel[0], bayer_pixel[bayer_line_step2], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			//rgb_pixel[rgb_line_step + 2] = bayer_pixel[line_step];

			// Bayer         -1 0 1
			//         0      r g r
			// line_step      g b G
			// line_step2     r g r
			rgb_buffer[rgb_line_step + 3] = AVG (bayer_pixel[1], bayer_pixel[bayer_line_step2 + 1]);
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];

			bayer_pixel += bayer_line_step + 2;
			rgb_buffer += rgb_line_step + 6 + rgb_line_skip;
		}

		//last two lines
		// Bayer         0 1 2
		//        -1     b g b
		//         0     G r g
		// line_step     b g b

		rgb_buffer[rgb_line_step + 3] = rgb_buffer[rgb_line_step ] = rgb_buffer[3] = rgb_buffer[0] = bayer_pixel[1]; // red pixel
		rgb_buffer[1] = bayer_pixel[0]; // green pixel
		rgb_buffer[rgb_line_step + 2] = rgb_buffer[2] = bayer_pixel[bayer_line_step]; // blue;

		// Bayer         0 1 2
		//        -1     b g b
		//         0     g R g
		// line_step     b g b
		//rgb_pixel[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
		rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[2 - bayer_line_step]);

		// BGBG line
		// Bayer         0 1 2
		//        -1     b g b
		//         0     g r g
		// line_step     B g b
		//rgb_pixel[rgb_line_step    ] = bayer_pixel[1];
		rgb_buffer[rgb_line_step + 1] = AVG (bayer_pixel[0], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

		// Bayer         0 1 2
		//        -1     b g b
		//         0     g r g
		// line_step     b G b
		//rgb_pixel[rgb_line_step + 3] = AVG( bayer_pixel[1] , bayer_pixel[line_step2+1] );
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);

		rgb_buffer += 6;
		bayer_pixel += 2;
		// rest of the last two lines
		for (xIdx = 2; xIdx < width - 2; xIdx += 2, rgb_buffer += 6, bayer_pixel += 2)
		{
			// GRGR line
			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r G r g
			// line_step    g b g b
			rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
			rgb_buffer[1] = bayer_pixel[0];
			rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g R g
			// line_step    g b g b
			rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
			rgb_buffer[4] = AVG4 (bayer_pixel[0], bayer_pixel[2], bayer_pixel[bayer_line_step + 1], bayer_pixel[1 - bayer_line_step]);
			rgb_buffer[5] = AVG4 (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2], bayer_pixel[-bayer_line_step], bayer_pixel[-bayer_line_step + 2]);

			// BGBG line
			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g r g
			// line_step    g B g b
			rgb_buffer[rgb_line_step ] = AVG (bayer_pixel[-1], bayer_pixel[1]);
			rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
			rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];


			// Bayer       -1 0 1 2
			//        -1    g b g b
			//         0    r g r g
			// line_step    g b G b
			//rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
			rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
			rgb_buffer[rgb_line_step + 5] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[bayer_line_step + 2]);
		}

		// last two pixel values for first two lines
		// GRGR line
		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r G r
		// line_step    g b g
		rgb_buffer[rgb_line_step ] = rgb_buffer[0] = AVG (bayer_pixel[1], bayer_pixel[-1]);
		rgb_buffer[1] = bayer_pixel[0];
		rgb_buffer[5] = rgb_buffer[2] = AVG (bayer_pixel[bayer_line_step], bayer_pixel[-bayer_line_step]);

		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g R
		// line_step    g b g
		rgb_buffer[rgb_line_step + 3] = rgb_buffer[3] = bayer_pixel[1];
		rgb_buffer[4] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step + 1], bayer_pixel[-bayer_line_step + 1]);
		//rgb_pixel[5] = AVG( bayer_pixel[line_step], bayer_pixel[-line_step] );

		// BGBG line
		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g r
		// line_step    g B g
		//rgb_pixel[rgb_line_step    ] = AVG2( bayer_pixel[-1], bayer_pixel[1] );
		rgb_buffer[rgb_line_step + 1] = AVG3 (bayer_pixel[0], bayer_pixel[bayer_line_step - 1], bayer_pixel[bayer_line_step + 1]);
		rgb_buffer[rgb_line_step + 5] = rgb_buffer[rgb_line_step + 2] = bayer_pixel[bayer_line_step];

		// Bayer       -1 0 1
		//        -1    g b g
		//         0    r g r
		// line_step    g b G
		//rgb_pixel[rgb_line_step + 3] = bayer_pixel[1];
		rgb_buffer[rgb_line_step + 4] = bayer_pixel[bayer_line_step + 1];
		//rgb_pixel[rgb_line_step + 5] = bayer_pixel[line_step];
	}

	return  RET_OK;
}


unsigned long Kinect::SaveParameters(const char* filename) 
{
	return ipa_Utils::RET_OK;
}


unsigned long Kinect::LoadParameters(const char* filename, int cameraIndex)
{
	//return ipa_Utils::RET_FAILED;

	//XnStatus retVal = XN_STATUS_OK;
	openni::Status retVal = openni::STATUS_OK;

	// Load Kinect parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - Kinect::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << filename << "'" << std::endl;

	std::string tempString;
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
//	BEGIN LibCameraSensors->PMDCamCube
//************************************************************************************
			// Tag element "Swissranger3 of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_SR31 = NULL;
			std::stringstream ss;
			ss << "Kinect_" << cameraIndex;
			p_xmlElement_Root_SR31 = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_SR31 )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->Kinect->OpenNI
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				/*
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "OpenNI" );
				if ( p_xmlElement_Child )
				{
					TiXmlPrinter printer;
					printer.SetIndent( "    " );
					p_xmlElement_Child->Accept( &printer );
					std::string xmltext = printer.CStr();

					// Write configuration to the temporary file.
					// This is a hack, because there is a bug in RunXmlScript().
					// TODO: remove hack when bug in RunXmlScript() will be fixed.
					char xmlFilename[] = "kinect_xml_tmp.xml";
					std::ofstream outfile( xmlFilename );
					outfile.write(xmltext.c_str(), xmltext.length());
					outfile.close();

					retVal = m_context.RunXmlScriptFromFile( xmlFilename );

					// Remove temporary configuration file.
					remove( xmlFilename );

					//retVal = m_context.RunXmlScript( xmltext.c_str() );

					if (retVal !=XN_STATUS_OK)
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Error while reading openNI parameters" << std::endl;
						std::cerr << "\t ... " << xnGetStatusName(retVal) << std::endl;
						return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
					}
				}
				else
				{
					std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'OpenNI'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			*/

//************************************************************************************
//	BEGIN LibCameraSensors->Kinect->Role
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "Role" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Role'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_RangeCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_RangeCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}

				}
				else
				{
					std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Role'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Kinect->VideoFormat
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "VideoFormat" );
				std::string tempString;
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "type", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'type' of tag 'VideoFormat'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "SXGA") 
					{
						m_ColorCamVideoFormat = SXGA;
					}
					else if (tempString == "VGA")
					{
						m_ColorCamVideoFormat = VGA;
					}
					else
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Video format " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'VideoFormat'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->Kinect->CalibrationMethod
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "CalibrationMethod" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "name", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'name' of tag 'CalibrationMethod'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "MATLAB") 
					{
						m_CalibrationMethod = MATLAB;
					}
					else if (tempString == "MATLAB_NO_Z")
					{
						m_CalibrationMethod = MATLAB_NO_Z;
					}
					else if (tempString == "NATIVE") 
					{
						m_CalibrationMethod = NATIVE;
					}
					else
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CalibrationMethod'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
//************************************************************************************
//	BEGIN LibCameraSensors->Kinect->OffsetZ
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "OffsetZ" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "dz", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'dz' of tag 'CalibrationMethod'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					std::stringstream ss_temp(tempString);
					ss_temp >> m_dZ;
				}
				else
				{
					std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'OffsetZ'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->Kinect
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - Kinect::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "\t [OK] Parsing xml calibration file\n";

	return RET_OK;
}
