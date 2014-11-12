/// @file ICCam.h
///
/// <!-- Color sensors -->
/// <ICCam_0>
///   <SerialNumber id="ISB3200016679" />
///   
///   <!-- Valid types: DFx 41F02 and DBx 31AF03 -->
///   <CameraType type="DBx 31AF03" />
/// </ICCam_0>
/// Interface to ImagingSource camera DBK 31AF03. Implementation depends on the
/// DirectShow SDK from Microsoft for Windows.
/// @author Jan Fischer
/// @date May, 2008.

#ifndef __IPA_ICCAM_H__
#define __IPA_ICCAM_H__

#ifdef __LINUX__
//****************************************************************************************
// LINUX
//****************************************************************************************

#include "cob_camera_sensors/AbstractColorCamera.h"
#include "cob_camera_sensors_ipa/UnicapCamera.h"

// Properties of camera -> Place in xml file

const int DFK_NO_OF_LAYERS = 3; 		///Number of layers
const int DFK_IMG_DEPTH = IPL_DEPTH_8U; 	/// Depth of DFK image
const int DFK_FOCAL_LENGTH = 8; //mm
const int DFK_MAX_RESOLVABLE_DISTANCE = 7500; //mm
const int DFK_MIN_RESOLVABLE_DISTANCE = 750; //mm



namespace ipa_CameraSensors {

/// @ingroup ColorCameraDriver
/// Interface to Imaging Source cameras
class __DLL_LIBCAMERASENSORS__ ICCam : public AbstractColorCamera
{
private:
	UnicapCamera* m_unicapCamera;
	std::string m_serialNumber;
	std::string m_cameraType;

	//unsigned long LoadParameters(const char* filename, int cameraIndex);
	unsigned long SetParameters() {return RET_OK;}

public:

	 
	/// Constructor
	ICCam();
	ICCam(int res);
	
	 /// Destructor.
	~ICCam();

	//*******************************************************************************
	// AbstractColorCamera interface implementation
	//*******************************************************************************
	
	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	//int Open(int index = 0, int format =MODE_F7_1 , int color_mode =COLOR_FORMAT_5, 
	//						int frame_rate = FRAMERATE_30);
	unsigned long Close();

	unsigned long GetColorImage(cv::Mat* img, bool getLatestFrame);

	unsigned long SaveParameters(const char* filename);		//speichert die Parameter in das File
	
	unsigned long SetProperty(t_cameraProperty* cameraProperty); 
	unsigned long SetPropertyDefaults();

	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long PrintCameraInformation();
	unsigned long TestCamera(const char* filename);

 //private:

	unsigned long LoadParameters(const char* filename, int cameraIndex);

};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_ICCam();

} // end namespace
#else
//****************************************************************************************
// WINDOWS
//****************************************************************************************

#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#include <tisudshl.h> 

using namespace _DSHOWLIB_NAMESPACE;

namespace ipa_CameraSensors {

static const int ICCAM_COLUMNS = 1024;
static const int ICCAM_ROWS = 768;

/// @ingroup ColorCameraDriver
/// Interface to Imaging Source cameras
class __DLL_LIBCAMERASENSORS__ ICCam : public AbstractColorCamera
{
	public:
		/// Constructor.
		/// The Constructor initializes the frame grabber, configures the camera device
		/// and sets up the frame storage.
		ICCam();
		~ICCam();

		//*******************************************************************************
		// AbstractColorCamera interface implementation
		//*******************************************************************************
		
		unsigned long Init(std::string directory, int cameraIndex = 0);

		unsigned long Open();
		//int Open(int index = 0, int format =MODE_F7_1 , int color_mode =COLOR_FORMAT_5, 
		//						int frame_rate = FRAMERATE_30);
		unsigned long Close();

		unsigned long GetColorImage(cv::Mat* img, bool getLatestFrame=true);
		
		unsigned long SaveParameters(const char* filename);		
		
		unsigned long SetProperty(t_cameraProperty* cameraProperty); 
		unsigned long SetPropertyDefaults();

		unsigned long GetProperty(t_cameraProperty* cameraProperty);	

		unsigned long PrintCameraInformation();
		unsigned long TestCamera(const char* filename);

	private:

		unsigned long LoadParameters(const char* filename, int cameraIndex);

		unsigned long SetParameters(){return ipa_CameraSensors::RET_OK;};

		//*******************************************************************************
		// Camera specific members
		//*******************************************************************************

		std::string m_serialNumber;
		std::string m_cameraType;
		int m_resolution[2];

		DShowLib::Grabber* m_grabber;
		DShowLib::tFrameHandlerSinkPtr m_pSink;
		DShowLib::FrameTypeInfo m_Info;
		cv::Size m_ColorImageResolution;
};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_ICCam();

} // end namespace
#endif // __LINUX__
#endif // __IPA_ICCAM_H__
