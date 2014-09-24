/// @file AxisCam.h
/// Interface to Axis 2100 Webcamera.
/// @author Jan Fischer
/// @date May 2008.

#ifndef __IPA_AXISCAM_H__
#define __IPA_AXISCAM_H__

#ifdef __LINUX__
	#include "cob_camera_sensors/AbstractColorCamera.h"
	#include "cob_camera_sensors/IPCamera.h"
#else
	#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
	#include "cob_driver_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/IPCamera.h"
#endif

namespace ipa_CameraSensors {

/// @ingroup ColorCameraDriver
/// Interface for Axis 2100 network camera
class __DLL_LIBCAMERASENSORS__ AxisCam : public AbstractColorCamera
{
	
	public:
		/// Constructor.
		AxisCam();
		~AxisCam();

		//*******************************************************************************
		// AbstractColorCamera interface implementation
		//*******************************************************************************
		
		unsigned long Init(std::string directory, int cameraIndex = 0);

		unsigned long Open();

		unsigned long Close();

		unsigned long GetColorImage(cv::Mat* colorImage, bool getLatestImage=true);

		unsigned long SaveParameters(const char* filename);		
		
		unsigned long SetProperty(t_cameraProperty* cameraProperty); 
		unsigned long SetPropertyDefaults();

		unsigned long GetProperty(t_cameraProperty* cameraProperty);	

		unsigned long PrintCameraInformation();
		unsigned long TestCamera(const char* filename);

	private:

		/// Parses the XML configuration file, that holds the camera settings
		/// @param filename The file name and path of the configuration file
		/// @param cameraIndex The index of the camera within the configuration file
		///		   i.e. AXIS_CAM_0 or AXIS_CAM_1
		/// @return Return value
		unsigned long LoadParameters(const char* filename, int cameraIndex);

		unsigned long SetParameters(){return RET_OK;};

		//*******************************************************************************
		// Camera specific members
		//*******************************************************************************

	public:

		IPCamera* m_IPCamera;

};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_AxisCam();

} // end namespace

#endif // __IPA_AXISCAM_H__
