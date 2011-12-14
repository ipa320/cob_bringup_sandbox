#include "../include/cob_camera_sensors_ipa/StdAfx.h"
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/Kinect.h"
	#include "tinyxml.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
	#include "cob_vision/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/Kinect.h"
	#include "cob_vision/windows/src/extern/TinyXml/tinyxml.h"
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

#include <fstream>
#include <iostream>

using namespace ipa_CameraSensors;
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_Kinect()
{
	return AbstractRangeImagingSensorPtr(new Kinect());
}

Kinect::Kinect()
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;
	
	m_CoeffsInitialized = false;
}

Kinect::~Kinect()
{
	if (isOpen())
	{
		m_context.StopGeneratingAll();
		m_context.Shutdown ();
	}
}


unsigned long Kinect::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return ipa_Utils::RET_OK;
	}

	XnStatus retVal = XN_STATUS_OK;

	retVal = m_context.Init(); 
	if (retVal !=XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot init openNI context" << std::endl;
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
	retVal = m_depth_generator.Create(m_context);
	if (retVal !=XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot create depth" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Create a ImageGenerator node 
	retVal = m_image_generator.Create(m_context);
	if (retVal !=XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot create color image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Set video format
	XnMapOutputMode output_mode;
	switch (m_ColorCamVideoFormat)
	{
	case SXGA:
		output_mode.nFPS = 15;
		output_mode.nXRes = XN_SXGA_X_RES;
		output_mode.nYRes = XN_SXGA_Y_RES;
		break;
	case VGA:
		output_mode.nFPS = 30;
		output_mode.nXRes = XN_VGA_X_RES;
		output_mode.nYRes = XN_VGA_Y_RES;
		break;
	default:
		output_mode.nFPS = 30;
		output_mode.nXRes = XN_VGA_X_RES;
		output_mode.nYRes = XN_VGA_Y_RES;
	}

	retVal = m_image_generator.SetMapOutputMode (output_mode);
	if (retVal !=XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot set output mode for color image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Maximal resolution for depth camera
	output_mode.nFPS = 30;
	output_mode.nXRes = XN_VGA_X_RES;
	output_mode.nYRes = XN_VGA_Y_RES;

	retVal = m_depth_generator.SetMapOutputMode (output_mode);
	if (retVal !=XN_STATUS_OK)
	{
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Cannot Cannot set output mode for depth image" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// Adjust the viewpoint of the range data according to the image data
	retVal = m_depth_generator.GetAlternativeViewPointCap().SetViewPoint( m_image_generator );
    if (retVal != XN_STATUS_OK)
    {
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Error in depth stream to color image registration";
		return ipa_Utils::RET_FAILED;
    }

	// Set input format to uncompressed 8-bit BAYER
	retVal = m_image_generator.SetIntProperty ("InputFormat", 6);
	if (retVal != XN_STATUS_OK)
    {
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not set uncompressed 8-bit BAYER format";
		return ipa_Utils::RET_FAILED;
    }

	// Bypass camera internal debayering give raw bayer pattern
	retVal = m_image_generator.SetPixelFormat (XN_PIXEL_FORMAT_GRAYSCALE_8_BIT);
	if (retVal != XN_STATUS_OK)
    {
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not bypass camera internal debayering";
		return ipa_Utils::RET_FAILED;
    }

	// RegistrationType should be 2 (software) for Kinect, 1 (hardware) for PS
	//retVal = m_depth_generator.SetIntProperty("RegistrationType", 2);
	//if (retVal != XN_STATUS_OK)
 //   {
	//	std::cerr << "ERROR - Kinect::Init:" << std::endl;
	//	std::cerr << "\t ... Could not set registration type";
	//	return ipa_Utils::RET_FAILED;
 //   }

	retVal = m_depth_generator.GetIntProperty( "ShadowValue", m_shadowValue );
	if (retVal != XN_STATUS_OK)
    {
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not read shadow value";
		return ipa_Utils::RET_FAILED;
    }
      
	retVal = m_depth_generator.GetIntProperty( "NoSampleValue", m_noSampleValue );
	if (retVal != XN_STATUS_OK)
    {
		std::cerr << "ERROR - Kinect::Init:" << std::endl;
		std::cerr << "\t ... Could not read sample values for invalid disparities";
		return ipa_Utils::RET_FAILED;
    }

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
		
	XnStatus retVal = XN_STATUS_OK;

	std::cout << "INFO - Kinect::Open():" << std::endl;
	std::cout << "\t ... Opening camera device" << std::endl;
	std::cout << "\t ... This may take some seconds" << std::endl;

	retVal = m_context.StartGeneratingAll(); 
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
	m_range_mat.create(height, width, CV_16UC1);

	std::cout << "*************************************************" << std::endl;
	std::cout << "Kinect::Open: Kinect camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long Kinect::Close()
{
	m_context.StopGeneratingAll();

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
			cameraProperty->cameraResolution.xResolution = (int) m_image_md.XRes();
			cameraProperty->cameraResolution.yResolution = (int) m_image_md.YRes();
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

	int color_width = m_image_md.XRes();
	int color_height = m_image_md.YRes();

	if(rangeImage)
	{
		// Depth image is upsampled according to the size of the color image
		rangeImage->create(color_height, color_width, CV_32FC1);
		rangeImageData = rangeImage->ptr<char>(0);
		widthStepRange = rangeImage->step;
	}

	if(colorImage)
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
	int color_width = m_image_md.XRes();
	int color_height = m_image_md.YRes();
	int range_width = m_depth_md.XRes();
	int range_height = m_depth_md.YRes();

	m_depth_generator.WaitAndUpdateData();
	m_depth_generator.GetMetaData(m_depth_md);
	m_image_generator.WaitAndUpdateData();
	m_image_generator.GetMetaData(m_image_md);

	//// TODO: Use real calibration data
	//float scale = color_width / (float)XN_SXGA_X_RES;
	//// focal length for regular camera producing color images in native SXGA mode
	//float rgb_focal_length_SXGA_ = 1050; 
	//// depth values are registered to color camera
	//float depth_registered_focal_length =  rgb_focal_length_SXGA_ * scale;
	//float constant = 0.001 / depth_registered_focal_length;
	//float cx = (color_width >> 1 ) - 0.5f;
	//float cy = (color_height >> 1) - 0.5f;

	double fx, fy, cx, cy;
	float constant;
	fx = m_intrinsicMatrix.at<double>(0, 0);
	fy = m_intrinsicMatrix.at<double>(1, 1);
	cx = m_intrinsicMatrix.at<double>(0, 2);
	cy = m_intrinsicMatrix.at<double>(1, 2);
	constant = 0.001 / fx;

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
		memcpy( range_mat_ptr, m_depth_md.Data(), range_height * range_width * sizeof(XnDepthPixel) );

		// Resize if necessary
		cv::Mat resized_range_mat;
		if (m_ColorCamVideoFormat == SXGA)
		{
			resized_range_mat.create(XN_SXGA_Y_RES, XN_SXGA_X_RES, m_range_mat.type());
			resized_range_mat.setTo(cv::Scalar(m_badDepth));
			cv::Mat sub_mat = resized_range_mat.rowRange(0, 2*XN_VGA_Y_RES);
			cv::resize(m_range_mat, sub_mat, cv::Size(), 2, 2, CV_INTER_NN);
		}
		else
			resized_range_mat = m_range_mat;
		
		// Convert zuv values to float xyz
		if (createXYZImage)
		{
			int xzy_step = color_width * sizeof(float) * 3;
			int d_step = color_width * sizeof(unsigned short);

			for(unsigned int row=0; row<(unsigned int)color_height; row++)
			{
				p_xyz = (float*) (cartesianImageData + row * xzy_step);
				p_us_dist = resized_range_mat.ptr<unsigned short>(row);

				for (unsigned int col = 0; col < (unsigned int)color_width; col++)
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
						p_xyz[colTimes3] = (col - cx) * p_us_dist[col] * constant;
						p_xyz[colTimes3 + 1] = (row - cy) * p_us_dist[col] * constant;
						p_xyz[colTimes3 + 2] = p_us_dist[col] * 0.001;

						if (p_xyz[colTimes3 + 2] < 0)
						{
							std::cout << "<0: " << row << " " << col << "\n";
						}

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
		}
		// Convert z values to float z
		if (createRangeImage)
		{
			// Get range values
			unsigned short us_val = 0;
			int d_step = color_width * sizeof(float);

			for(unsigned int row=0; row<(unsigned int)color_height; row++)
			{
				p_f_dist = (float*)(rangeImageData + row * d_step);
				p_us_dist = resized_range_mat.ptr<unsigned short>(row);

				for (unsigned int col=0; col<(unsigned int)color_width; col++)
				{
					us_val = p_us_dist[col];
					// Convert to float to stay consistent with other range cameras
					if (us_val != m_noSampleValue && 
						us_val != m_shadowValue &&
						us_val != 0)
						p_f_dist[col] = (float)us_val;
					else
						p_f_dist[col] = (float)m_badDepth;
				}	
			}
		}
	}

	// Read out the color image
	// Check if new data is available
	if (colorImageData)
    {
		FillRGB(color_width, color_height, ( unsigned char*) colorImageData);

		// Switch red and blue image channels
		unsigned char temp_val = 0;
		unsigned char* p_dest = 0;
		unsigned int colTimes3 = 0;
		for(unsigned int row=0; row<(unsigned int)color_height; row++)
		{
			p_dest = ((unsigned char*)colorImageData) + row * color_width * 3;
			for (unsigned int col=0; col<(unsigned int)color_width; col++)
			{
				colTimes3 = col*3;
				temp_val = p_dest[colTimes3];
				p_dest[colTimes3] = p_dest[colTimes3 + 2];
				p_dest[colTimes3 + 2] = temp_val;
			}	
		}
	}

	return  RET_OK;
}

unsigned long Kinect::FillRGB(unsigned width, unsigned height, unsigned char* rgb_buffer)
{
	if (width != m_image_md.XRes () || height != m_image_md.YRes ())
		// TODO: Throw exception
		return RET_FAILED;
	
	unsigned int rgb_line_step = width * 3;

	// padding skip for destination image
	unsigned rgb_line_skip = rgb_line_step - width * 3;

	register const XnUInt8 *bayer_pixel = m_image_md.Data ();
	register unsigned yIdx, xIdx;

	int bayer_line_step = m_image_md.XRes ();
	int bayer_line_step2 = m_image_md.XRes () << 1;

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
	XnStatus retVal = XN_STATUS_OK;

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
//	BEGIN LibCameraSensors->Kinect->CalibrationMethod
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
