///
/// @file UnicapCamera.h
///
/// This class defines an interface to the uniform API for image acquisition devices.
/// @author Jan Fischer
/// @date August, 2008
///

#ifndef __IPA_UNICAPCAMERA_H__
#define __IPA_UNICAPCAMERA_H__

#ifdef __LINUX__
	#include "cob_vision_utils/CameraSensorDefines.h"
#else
	#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorDefines.h"
#endif

// Include class for representing image
#include <errno.h>
#include <unicap.h>
#include <unicap/unicap_status.h>

#define FOUR_CC_BAYER 808466521
#define FOUR_CC_UYVY 1498831189

#define MODE_BAYER 1
#define MODE_UYVY 0

//names of the structures in the calibration storage
#define STORAGE_ID_COEFFS_A "ACoeffs"
#define STORAGE_ID_COEFFS_B "BCoeffs"
#define STORAGE_ID_OFFSET_U0 "OffsetU0"
#define STORAGE_ID_OFFSET_V0 "OffsetV0"
#define STORAGE_ID_MAGN_ALPHA "MagnA"
#define STORAGE_ID_MAGN_BETA "MagnB"


/** @ingroup ColorCameraDriver 
 * Low-level class to firewire cameras. This class is a wrapper of the unicap library,
 * which in turn constitutes a wrapper of libcv.
 */
class __DLL_LIBCAMERASENSORS__ UnicapCamera
{
public:

	 
	/**
	 * Constructor
	 */
	UnicapCamera();
	UnicapCamera(int res);
	
	/**
	 * Destructor
	 *
	 */
	~UnicapCamera(); 
	
	bool IsCameraActive() {return m_CameraActive;}

	
	/**
	 * Initializes the sensor.
	 * @return 0 if successful, other value indicates failure
	 */
	int Open();
	
	/**
	 * Uninitializes the sensor
	 * @return 0 if successful, other value indicates failure
	 */
	int Close(void);
	
	
	//------------ Access Camera Parameters

	int ShowAvailableVideoFormats();

	int SetColorMode(int colorMode);

	//------------ Getting image
	/**
	 * Acquires exactly one image from the sensor (not averaged)
	 * @param Img IplImage structure the acuired image is written into
	 * @param Filename If specified, the image is saved under this file name
	 * @return 0 if successful, other value indicates failure
	 */
	int GetColorImage(cv::Mat* img, char* FileName = NULL);
	
	int PrintCameraInformation();
	
	int SetProperty(int property, int value );

	int ConvertImage(cv::Mat* inputImg, unicap_data_buffer_t * inputRawBufferData);
	
	int ConvRGBIplImage(cv::Mat* Img, unicap_data_buffer_t * rawBufferData);
	
	int ConvUYVY2IplImage(cv::Mat* Img, unicap_data_buffer_t * rawBufferData);
	
	

	
	// ------------- error values
	enum
	{
		OK	= 0,
		ERROR_NO_CAMERAS_FOUND = -1,
		ERROR_CAMERA_COULD_NOT_BE_OPENED = -2,
		ERROR_NOT_OPENED = -3,
 		ERROR_NOT_CALIBRATED = -4,
		ERROR_NO_MEMORY = -5,
		ERROR_NO_FORMAT_SET = -6,
		UNSPECIFIED_ERROR = -10
	} UNICAPCAMERA_ERRORS;


protected:	//#################
		//ich habe aus private, protected gemacht

	unicap_handle_t * m_Handle;			///Camera Handle
	unicap_device_t * m_Device;
	unicap_format_t * m_Format;

        

	bool m_CameraActive;
	int m_Resolution;			///Video Format ID (from Camera)
	int m_ColorMode;
};



#endif // __IPA_UNICAPCAMERA_H__
#endif // __LINUX__

