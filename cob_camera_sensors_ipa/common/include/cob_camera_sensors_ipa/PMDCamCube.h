/// @file PMDCamCube.h
/// Platform independent interface to PMDCamCube camera.
/// Implementation depends on PMD library.
/// @author Jan Fischer
/// @date 200

#ifndef __IPA_PMDCAMCUBE_H__
#define __IPA_PMDCAMCUBE_H__

#ifdef __LINUX__
	#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#else
	#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#endif

#include <pmdsdk2.h>

#ifdef __LINUX__
	/// Name of plugin to access the camera
	#define SOURCE_PLUGIN "camcube.L32.pap"
	/// Name of plugin for range (x,y,z) calculations
	#define PROC_PLUGIN "camcubeproc.L32.ppp"
#else
	/// Name of plugin to access the camera
	#define SOURCE_PLUGIN "camcube.W32.pap"
	/// Name of plugin for range (x,y,z) calculations
	#define PROC_PLUGIN "camcubeproc.W32.ppp"
#endif

namespace ipa_CameraSensors {

/// @ingroup RangeCameraDriver
/// Interface class to CamCube camera.
/// Platform independent interface to PMD CamCube camera.
/// Implementation depends on CamCube SDK2.
class __DLL_LIBCAMERASENSORS__ PMDCamCube : public AbstractRangeImagingSensor
{
public:

	PMDCamCube();
	~PMDCamCube();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* RangeImage=NULL, char* IntensityImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY_32F1);
	unsigned long AcquireImages(cv::Mat* rangeImage = 0, cv::Mat* intensityImage = 0,
		cv::Mat* cartesianImage = 0, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY_32F1);

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

private:
	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************

	unsigned long GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated);
	unsigned long GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y);

	/// Load general SR31 parameters and previously determined calibration parameters.
	/// @param filename Swissranger parameter path and file name.
	/// @param cameraIndex The index of the camera within the configuration file
	///		   i.e. SR_CAM_0 or SR_CAM_1
	/// @return Return value 
	unsigned long LoadParameters(const char* filename, int cameraIndex);

	/// Parses the data extracted by <code>LoadParameters</code> and calls the
	/// corresponding <code>SetProperty</code> functions.
	/// @return Return code
	unsigned long SetParameters();

	PMDHandle m_PMDCam; 		///< Handle to USB PMD camera
	std::string m_Serial;		///< Serial number of the PMD camera

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
__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr CreateRangeImagingSensor_PMDCam();

} // End namespace ipa_CameraSensors
#endif // __IPA_PMDCAMCUBE_H__


