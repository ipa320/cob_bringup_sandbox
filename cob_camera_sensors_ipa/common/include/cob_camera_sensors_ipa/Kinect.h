// Drivers for kinect camera:
// https://github.com/avin2/SensorKinect 
// follow instructions in the "README" File under Installation Notes 
//
// OpenNI interfaces:
// http://www.openni.org/downloadfiles/openni-binaries/20-latest-unstable
//
// NITE
// http://www.openni.org/downloadfiles/12-openni-compliant-middleware-binaries
//
// OpenNI Compliant Hardware Binaries
// http://www.openni.org/downloadfiles/30-openni-compliant-hardware-binaries
//
// Implementation examples
// http://www.ros.org/wiki/ni
//
// OpenNI specific XML file example for Kinect camera e.g.:
//<OpenNI>
//	<Licenses>
//		<License vendor="PrimeSense" key="0KOIk2JeIBYClPWVnMoRKn5cdY4="/>
//	</Licenses>
//	<Log writeToConsole="true" writeToFile="false">
//		<!-- 0 - Verbose, 1 - Info, 2 - Warning, 3 - Error (default) -->
//		<LogLevel value="3"/>
//		<Masks>
//			<Mask name="ALL" on="false"/>
//		</Masks>
//		<Dumps>
//		</Dumps>
//	</Log>
//	<ProductionNodes>
//		<Node type="Depth">
//			<Configuration>
//				<MapOutputMode xRes="640" yRes="480" FPS="30"/>
//				<Mirror on="true"/>
//			</Configuration>
//		</Node>
//		<Node type="Image">
//			<Configuration>
//				<MapOutputMode xRes="640" yRes="480" FPS="30"/>
//				<Mirror on="true"/>
//			</Configuration>
//		</Node>
//	</ProductionNodes>
//</OpenNI>
//
//---------------------------------------------------------------------
// the "main" calling the kinect could look like this:
//
//int main(int argc, char* argv[])
//{
//	ipa_CameraSensors::Kinect kinect;
//	kinect.Init("C:\kinecttests\ ",1);
//	kinect.Open();
//	cv::Mat kinectRangeimage;	
//	cv::Mat kinectRangeimage_8U3;
//	cv::Mat kinectColorimage;	
//
//	int cmd = 0;
//	while (cmd != 'q')
//	{
//		kinect.AcquireImages(&kinectRangeimage,&kinectColorimage);//, cv::Mat* grayImage, cv::Mat* cartesianImage,		bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
//	
//		ipa_Utils::ConvertToShowImage(kinectRangeimage, kinectRangeimage_8U3);
//		cv::imshow("Range", kinectRangeimage_8U3);
//		cv::imshow("Color", kinectColorimage);
//		cmd = cv::waitKey(20);
//	}
//}


#ifndef __IPA_KINECT_H__
#define __IPA_KINECT_H__

#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#else
	#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#endif

//#include <stdio.h>
//#include <math.h>
//#include <assert.h>
//#include <sstream>
//
#include <XnOS.h>
#include <XnCppWrapper.h>

namespace ipa_CameraSensors {

/// @ingroup RangeCameraDriver
/// Platform independent interface to Microsofts Kinect camera.
class __DLL_LIBCAMERASENSORS__ Kinect : public AbstractRangeImagingSensor
{
public:

	enum t_KinectColorCamVideoFormat
	{
		SXGA = 0, // 1280×1024
		VGA //640x480
	};

	Kinect();
	~Kinect();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepRange, int widthStepColor, int widthStepCartesian, char* RangeImage=NULL, char* IntensityImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY_32F1);
	unsigned long AcquireImages(cv::Mat* rangeImage = 0, cv::Mat* intensityImage = 0,
		cv::Mat* cartesianImage = 0, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY_32F1);

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

private:

	unsigned long FillRGB(unsigned width, unsigned height, unsigned char* rgb_buffer);

	cv::Mat m_range_mat; ///< Temporary storage

	xn::Context m_context;					// OpenNI main object
	xn::DepthGenerator m_depth_generator;	// OpenNI depthmap generator
	xn::ImageGenerator m_image_generator;	// OpenNI image generator

	xn::DepthMetaData m_depth_md; ///< OpenNI metadat for depth generator
	xn::ImageMetaData m_image_md; ///< OpenNI metadat for image generator
	
    XnDouble m_baseline; ///< The distance between the IR projector and the IR camera
	XnUInt64 m_depthFocalLength_VGA; ///< Focal length in pixels for the IR camera in VGA resolution
    XnUInt64 m_shadowValue; ///< The value for shadow (occluded pixels)

    XnUInt64 m_noSampleValue; ///< The value for pixels without a valid disparity measurement

	static const unsigned short m_badDepth = 0; ///< Value to indicate bad depth

	t_KinectColorCamVideoFormat m_ColorCamVideoFormat; ///< Video format of color camera

	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	/// Load general SR31 parameters and previously determined calibration parameters.
	/// @param filename Swissranger parameter path and file name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return value 
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	bool m_CoeffsInitialized;	///< True, when m_CoeffsAx have been initialized

	/// Given a 32 bit swissranger depth value, the real depth value in meteres is given by:
	/// z(u,v)=a0(u,v)+a1(u,v)*d(u,v)+a2(u,v)*d(u,v)^2
	///       +a3(u,v)*d(u,v)^3+a4(u,v)*d(u,v)^4+a5(u,v)*d(u,v)^5
	///       +a6(u,v)*d(u,v)^6;
	cv::Mat m_CoeffsA0; ///< a0 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA1; ///< a1 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA2; ///< a2 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA3; ///< a3 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA4; ///< a4 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA5; ///< a5 z-calibration parameters. One matrix entry corresponds to one pixel
	cv::Mat m_CoeffsA6; ///< a6 z-calibration parameters. One matrix entry corresponds to one pixel
};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr CreateRangeImagingSensor_Kinect();

} // End namespace ipa_CameraSensors
#endif // __IPA_KINECT_H__


