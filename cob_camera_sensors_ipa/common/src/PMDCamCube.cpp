#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/PMDCamCube.h"

	#include "cob_vision_utils/VisionUtils.h"
	#include "tinyxml.h"
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/PMDCamCube.h"

	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
#endif



using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_PMDCam()
{
	return AbstractRangeImagingSensorPtr(new PMDCamCube());
}

PMDCamCube::PMDCamCube()
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;
	
	m_PMDCam = 0;

	m_CoeffsInitialized = false;
}


PMDCamCube::~PMDCamCube()
{
	if (isOpen())
	{
		Close();
	}
}


unsigned long PMDCamCube::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	m_CameraType = ipa_CameraSensors::CAM_PMDCAM;

	// Load SR parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - PMDCamCube::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return (RET_FAILED | RET_INIT_CAMERA_FAILED);	
	}
	
	m_CoeffsInitialized = true;
	if (m_CalibrationMethod == MATLAB)
	{
		// Load z-calibration files
		std::string filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA0.xml";
		CvMat* c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA0.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA0 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA1.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA1.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA1 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA2.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA2.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA2 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA3.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA3.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA3 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA4.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA4.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA4 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA5.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA5.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA5 = c_mat;
			cvReleaseMat(&c_mat);
		}

		filename = directory + "MatlabCalibrationData/PMD/ZCoeffsA6.xml";
		c_mat = (CvMat*)cvLoad(filename.c_str()); 
		if (! c_mat)
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Error while loading " << directory + "MatlabCalibrationData/ZcoeffsA6.txt" << "." << std::endl;
			std::cerr << "\t ... Data is necessary for z-calibration of swissranger camera" << std::endl;
			m_CoeffsInitialized = false;
			// no RET_FAILED, as we might want to calibrate the camera to create these files
		}
		else
		{
			m_CoeffsA6 = c_mat;
			cvReleaseMat(&c_mat);
		}
	}

	// set init flag
	m_initialized = true;

	return RET_OK;
}


unsigned long PMDCamCube::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	int ret = 0;
	char err[128];
	
	std::cout << "INFO - PMDCamCube::Open():" << std::endl;
	std::cout << "\t ... Opening camera device" << std::endl;
	std::cout << "\t ... This may take a while " << std::endl;

	if (m_Serial == "0.0")
	{
		// Open first camera on bus
		ret = pmdOpen (&m_PMDCam, SOURCE_PLUGIN, "", PROC_PLUGIN, "");
	}
	else
	{
		// Open specified camera
		ret = pmdOpen (&m_PMDCam, SOURCE_PLUGIN, m_Serial.c_str(), PROC_PLUGIN, "");
	}
	if (ret != PMD_OK)
	{
		pmdGetLastError (0, err, 128);
		std::cerr << "ERROR - PMDCamCube::Open():" << std::endl;
		std::cerr << "\t ... Could not connect to PMD CamCube camera" << std::endl;
		std::cerr << "\t ... Check connection and specified serial number" << std::endl;
		std::cerr << "\t ... '" << err << "'" << std::endl;
		return RET_FAILED;
	}

	if (SetParameters() & ipa_CameraSensors::RET_FAILED)
	{
		std::cerr << "ERROR - AVTPikeCam::Open:" << std::endl;
		std::cerr << "\t ... Could not set parameters" << std::endl;
		return RET_FAILED;
	}

	std::cout << "INFO - PMDCamCube::Open():" << std::endl;
	char serial[16];
	ret = pmdSourceCommand (m_PMDCam, serial, 16, "GetSerialNumber");
	if (ret == PMD_OK)
	{
		std::cout << "\t ... Serial number: '" << serial << "'" << std::endl;
	}
	char offset[16];
	ret = pmdSourceCommand (m_PMDCam, offset, 16, "GetSoftOffset");
	if (ret == PMD_OK)
	{
		std::cout << "\t ... Soft offset: '" << offset << "'" << std::endl;
	}
	char roi[64];
	ret = pmdSourceCommand (m_PMDCam, roi, 64, "GetROI");
	if (ret == PMD_OK)
	{
		std::cout << "\t ... ROI (<left> <top> <width> <height>): '" << roi << "'" << std::endl;
	}
	unsigned int integrationTime = -1;
	ret = pmdGetIntegrationTime (m_PMDCam, &integrationTime, 0);
	if (ret == PMD_OK)
	{
		std::cout << "\t ... Integration time: '" << integrationTime << "' micro sec" << std::endl;
	}
	unsigned int modulationFrequency = -1;
	ret = pmdGetModulationFrequency (m_PMDCam, &modulationFrequency, 0);
	if (ret == PMD_OK)
	{
		std::cout << "\t ... Modulation frequency '" << modulationFrequency << "'" << std::endl;
	}

	std::cout << "*************************************************" << std::endl;
	std::cout << "PMDCamCube::Open: PMD CamCube camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long PMDCamCube::Close()
{
	char err[128];

	if (!isOpen())
	{
		return (RET_OK);
	}

	if (pmdClose(m_PMDCam) != PMD_OK)
	{
		pmdGetLastError (0, err, 128);
		std::cerr << "ERROR - PMDCamCube::Close():" << std::endl;
		std::cerr << "\t ... Could not close PMD CamCube camera" << std::endl;
		std::cerr << "\t ... '" << err << "'" << std::endl;
		return RET_FAILED;
	}
	m_PMDCam = 0;

	m_open = false;
	return RET_OK;

}


unsigned long PMDCamCube::SetProperty(t_cameraProperty* cameraProperty) 
{
	if (!m_PMDCam)
	{
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int ret = 0;
	char err[128];
	switch (cameraProperty->propertyID)
	{
		case PROP_INTEGRATION_TIME:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					ret = pmdSetIntegrationTime (m_PMDCam, 0, 2500);
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set integration time to AUTO mode" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					// Void
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
				if (cameraProperty->u_longData < 12 || cameraProperty->u_longData > 50000)
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Amplitude threshold must be between 12 and 50000" << std::endl;
					std::cerr << "\t ... Your value was '" << cameraProperty->u_longData << "'" << std::endl;
					return RET_FAILED;
				}
				else
				{
					unsigned int integrationTime = 0;
					ret = pmdGetValidIntegrationTime (m_PMDCam, &integrationTime, 0, CloseTo, cameraProperty->u_longData);
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
				
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not get closest integration time to'" << cameraProperty->u_longData << "'" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
					ret = pmdSetIntegrationTime (m_PMDCam, 0, integrationTime);
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set integration time to '" << integrationTime << "'" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		/*case PROP_MODULATION_FREQUENCY:
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_LAST);
					if(err<0)
					{
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set modulation frequency to AUTO mode" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					// Void
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_LONG | ipa_CameraSensors::TYPE_UNSIGNED))
			{
				// MF_40MHz, SR3k: maximal range 3.75m
				// MF_30MHz, SR3k, SR4k: maximal range 5m
				// MF_21MHz, SR3k: maximal range 7.14m
				// MF_20MHz, SR3k: maximal range 7.5m
				// MF_19MHz, SR3k: maximal range 7.89m
				// MF_60MHz, SR4k: maximal range 2.5m 
				// MF_15MHz, SR4k: maximal range 10m
				// MF_10MHz, SR4k: maximal range 15m
				// MF_29MHz, SR4k: maximal range 5.17m
				// MF_31MHz
				if (cameraProperty->stringData == "MF_40MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_40MHz);
				}
				else if (cameraProperty->stringData == "MF_30MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_30MHz);
				}
				else if (cameraProperty->stringData == "MF_21MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_21MHz);
				}
				else if (cameraProperty->stringData == "MF_20MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_20MHz);
				}
				else if (cameraProperty->stringData == "MF_19MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_19MHz);
				}
				else if (cameraProperty->stringData == "MF_60MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_60MHz);
				}
				else if (cameraProperty->stringData == "MF_15MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_15MHz);
				}
				else if (cameraProperty->stringData == "MF_10MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_10MHz);
				}
				else if (cameraProperty->stringData == "MF_29MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_29MHz);
				}
				else if (cameraProperty->stringData == "MF_319MHz")
				{
					err = SR_SetModulationFrequency(m_PMDCam, MF_31MHz);
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Modulation frequency " << cameraProperty->stringData << " unknown" << std::endl;
				}
				
				if(err<0)
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set modulation frequency " << cameraProperty->stringData << std::endl;
					return RET_FAILED;
				}
				
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_LONG|TYPE_UNSIGNED)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;*/
		case PROP_DISTANCE_OFFSET:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					std::string cmd = "SetSoftOffset 0.0";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set distance offset" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					// Void
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_STRING))
			{
				std::string cmd = "SetSoftOffset " + cameraProperty->stringData;
				ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
				if (ret != PMD_OK)
				{
					pmdGetLastError (0, err, 128);
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set distance offset" << std::endl;
					std::cerr << "\t ... '" << err << "'" << std::endl;
					return RET_FAILED;
				}	
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_STRING' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		case PROP_ROI:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					std::string cmd = "SetROI 0 0 0 0";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set ROI" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					// Void
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_STRING))
			{
				std::string cmd = "SetROI " + cameraProperty->stringData;
			
				ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
				if (ret != PMD_OK)
				{
					pmdGetLastError (0, err, 128);
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Could not set ROI" << std::endl;
					std::cerr << "\t ... '" << err << "'" << std::endl;
					return RET_FAILED;
				}	
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. '(TYPE_DATA|TYPE_STRING)' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		case PROP_EXPOSURE_TIME:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					std::string cmd = "SetExposureMode Normal";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set exposure mode" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					// Void
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_STRING))
			{
				if (cameraProperty->stringData == "SMB")
				{
					std::string cmd = "SetExposureMode SMB";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set exposure mode" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Exposure mode '" << cameraProperty->stringData << "' unknown" << std::endl;
				}
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_STRING' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		case PROP_LENS_CALIBRATION:	
			if (cameraProperty->propertyType & ipa_CameraSensors::TYPE_SPECIAL)
			{
				if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_AUTO)
				{
					std::string cmd = "SetLensCalibration On";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not set lens calibration" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else if(cameraProperty->specialValue == ipa_CameraSensors::VALUE_DEFAULT)
				{
					// Void
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Special value 'VALUE_AUTO' or 'VALUE_DEFAULT' expected." << std::endl;
					return RET_FAILED;
				}
			}
			else if (cameraProperty->propertyType & (ipa_CameraSensors::TYPE_STRING))
			{
				if (cameraProperty->stringData == "ON")
				{
					std::string cmd = "SetLensCalibration On";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not activate lens calibration" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else if (cameraProperty->stringData == "OFF")
				{
					std::string cmd = "SetLensCalibration Off";
					ret = pmdSourceCommand (m_PMDCam, 0, 0, cmd.c_str());
					if (ret != PMD_OK)
					{
						pmdGetLastError (0, err, 128);
						std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
						std::cerr << "\t ... Could not deactivate lens calibration" << std::endl;
						std::cerr << "\t ... '" << err << "'" << std::endl;
						return RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
					std::cerr << "\t ... Lens calibration toggle '" << cameraProperty->stringData << "' unknown" << std::endl;
				}
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::SetProperty:" << std::endl;
				std::cerr << "\t ... Wrong property type. 'TYPE_STRING' or special value 'TYPE_SPECIAL' expected." << std::endl;
				return RET_FAILED;
			}
			break;
		default: 				
			std::cout << "PMDCamCube::SetProperty: Property " << cameraProperty->propertyID << " unspecified.\n";
			return RET_FAILED;
			break;
	}

	return RET_OK;
}


unsigned long PMDCamCube::SetPropertyDefaults() 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}


unsigned long PMDCamCube::GetProperty(t_cameraProperty* cameraProperty) 
{
	int ret = 0;
	char err[128];
	PMDDataDescription dataDescriptor;
	switch (cameraProperty->propertyID)
	{
/*		case PROP_AMPLITUDE_THRESHOLD:
			if (isOpen())
			{
				cameraProperty->u_shortData = SR_GetAmplitudeThreshold(m_PMDCam);
				cameraProperty->propertyType = (TYPE_UNSIGNED | TYPE_SHORT);
			}
			else
			{
				return (RET_FAILED | RET_CAMERA_NOT_OPEN);
			}
			break;

		case PROP_INTEGRATION_TIME:	
			if (isOpen())
			{
				cameraProperty->u_charData = SR_GetIntegrationTime(m_PMDCam);
				cameraProperty->propertyType = (TYPE_UNSIGNED | TYPE_CHARACTER);	
			}
			else
			{
				return (RET_FAILED | RET_CAMERA_NOT_OPEN);
			}
			break;
			
		case PROP_ACQUIRE_MODE:	
			if (isOpen())
			{
				cameraProperty->integerData = SR_GetMode(m_PMDCam);
				cameraProperty->propertyType = TYPE_INTEGER;	
			}
			else
			{
				return (RET_FAILED | RET_CAMERA_NOT_OPEN);
			}
			break;
*/
		case PROP_DMA_BUFFER_SIZE:
			cameraProperty->u_integerData = m_BufferSize;
			return RET_OK;
			break;
		case PROP_CAMERA_RESOLUTION:
			if (isOpen())
			{
				ret = pmdGetSourceDataDescription (m_PMDCam, &dataDescriptor);
				if (ret != PMD_OK)
				{
					pmdGetLastError (m_PMDCam, err, 128);
					pmdGetLastError (0, err, 128);
					std::cerr << "ERROR - PMDCamCube::GetProperty:" << std::endl;
					std::cerr << "\t ... Could not get data description from camera" << std::endl;
					std::cerr << "\t ... '" << err << "'" << std::endl;
					return RET_FAILED;
				}
				cameraProperty->cameraResolution.xResolution = dataDescriptor.img.numColumns;
				cameraProperty->cameraResolution.yResolution = dataDescriptor.img.numRows;
				cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			}
			else
			{
				std::cout << "WARNING - PMDCamCube::GetProperty:" << std::endl;
				std::cout << "\t ... Camera not open" << std::endl;
				std::cout << "\t ... Resetting width and height to '204'x'204'" << std::endl;
				cameraProperty->cameraResolution.xResolution = 204;
				cameraProperty->cameraResolution.yResolution = 204;
				cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			}
			break;

		default: 				
			std::cout << "ERROR - PMDCamCube::GetProperty:" << std::endl;
			std::cout << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return RET_FAILED;
			break;

	}

	return RET_OK;
}


unsigned long PMDCamCube::AcquireImages(cv::Mat* rangeImage, cv::Mat* grayImage, cv::Mat* cartesianImage,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* grayImageData = 0;
	char* cartesianImageData = 0;
	int widthStepRange = -1;
	int widthStepGray = -1;
	int widthStepCartesian = -1;

	int width = -1;
	int height = -1;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	GetProperty(&cameraProperty);
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	if(rangeImage)
	{
		rangeImage->create(height, width, CV_32FC(1));
		rangeImageData = rangeImage->ptr<char>(0);
		widthStepRange = rangeImage->step;
	}

	if(grayImage)
	{
		grayImage->create(height, width, CV_32FC(1));
		grayImageData = grayImage->ptr<char>(0);
		widthStepGray = grayImage->step;
	}	

	if(cartesianImage)
	{
		cartesianImage->create(height, width, CV_32FC(3));
		cartesianImageData = cartesianImage->ptr<char>(0);
		widthStepCartesian = cartesianImage->step;
	}

	if (!rangeImage && !grayImage && !cartesianImage)
	{
		return RET_OK;
	}

	return AcquireImages(widthStepRange, widthStepGray, widthStepCartesian, rangeImageData, grayImageData,  cartesianImageData, getLatestFrame, undistort, grayImageType);
}

// Enables faster image retrival than AcquireImage
unsigned long PMDCamCube::AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImageData, char* grayImageData, char* cartesianImageData,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	int ret = 0;
	char err[128];
///***********************************************************************
// Get data from camera
///***********************************************************************
	if (!m_open)
	{
		std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
		std::cerr << "\t ... Camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	int width = -1;
	int height = -1;
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	cameraProperty.propertyID = PROP_CAMERA_RESOLUTION;
	GetProperty(&cameraProperty);
	width = cameraProperty.cameraResolution.xResolution;
	height = cameraProperty.cameraResolution.yResolution;

	// Acquire new image data
	ret = pmdUpdate (m_PMDCam);
	if (ret != PMD_OK)
	{
		pmdGetLastError (m_PMDCam, err, 128);
		std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
		std::cerr << "\t ... Could not get image data from camera" << std::endl;
		std::cerr << "\t ... '" << err << "'" << std::endl;
		return RET_FAILED;
	}
	
///***********************************************************************
// Range image (distorted or undistorted)
///***********************************************************************
	if (rangeImageData)
	{
		float* f_ptr = 0;
		float* f_ptr_dst = 0;
		cv::Mat pmdData(height, width, CV_32FC1 );

		ret = pmdGetAmplitudes(m_PMDCam, (float*) pmdData.data, height*width*sizeof(float));
		if (ret != PMD_OK)
		{
			pmdGetLastError (m_PMDCam, err, 128);
			std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
			std::cerr << "\t ... Could not get phase image data from camera" << std::endl;
			std::cerr << "\t ... '" << err << "'" << std::endl;
			return RET_FAILED;
		}
		
		// Left and right is mirrowed by PMD Cam
		for(unsigned int row=0; row<(unsigned int)height; row++)
		{
			f_ptr = pmdData.ptr<float>(row);
			f_ptr_dst = (float*) (rangeImageData + row*widthStepRange);

			for (unsigned int col=0; col<(unsigned int)width; col++)
			{
				f_ptr_dst[width - col - 1] = f_ptr[col];
			}	
		}

		if (undistort)
		{
			cv::Mat undistortedData (height, width, CV_32FC1, (float*) rangeImageData);
			cv::Mat distortedData;
 
			assert (!m_undistortMapX.empty() && !m_undistortMapY.empty());
			cv::remap(distortedData, undistortedData, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);
		}

	} // End if (rangeImage)

///***********************************************************************
// Gray image based on amplitude or intenstiy
///***********************************************************************
	if(grayImageData)
	{
		float* f_ptr = 0;
		float* f_ptr_dst = 0;
		cv::Mat pmdData(height, width, CV_32FC1 );
		
		if (grayImageType == ipa_CameraSensors::INTENSITY_32F1)
		{
			ret = pmdGetIntensities(m_PMDCam, (float*) pmdData.data, height*width*sizeof (float));
		}
		else
		{
			ret = pmdGetAmplitudes(m_PMDCam, (float*) pmdData.data, height*width*sizeof (float));
		}

		if (ret != PMD_OK)
		{
			pmdGetLastError (m_PMDCam, err, 128);
			std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
			std::cerr << "\t ... Could not get intensity image data from camera" << std::endl;
			std::cerr << "\t ... '" << err << "'" << std::endl;
			return RET_FAILED;
		}

		// Left and right is mirrowed by PMD Cam
		for(unsigned int row=0; row<(unsigned int)height; row++)
		{
			f_ptr = pmdData.ptr<float>(row);
			f_ptr_dst = (float*) (grayImageData + row*widthStepGray);

			for (unsigned int col=0; col<(unsigned int)width; col++)
			{
				f_ptr_dst[width - col -1] = f_ptr[col];
			}	
		}
		
		if (undistort)
		{
			cv::Mat undistortedData (height, width, CV_32FC1, (float*) grayImageData);
			cv::Mat distortedData;
 
			assert (!m_undistortMapX.empty() && !m_undistortMapY.empty());
			cv::remap(distortedData, undistortedData, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);
		}

	}

///***********************************************************************
// Cartesian image (always undistorted)
///***********************************************************************
	if(cartesianImageData)
	{
		float x = -1;
		float y = -1;
		float z = -1;

		float* zCalibratedPtr = 0;
		float* f_ptr = 0;
		float* f_ptr_dst = 0;

		if(m_CalibrationMethod==MATLAB)
		{
			// Not possible with CamCube
			/*if (m_CoeffsInitialized)
			{
				float zRaw = -1;
				// Get raw range (phase shift) data
				CvMat* distortedData = cvCreateMat( height, width, CV_32FC1 );
				ret = pmdGetAmplitudes(m_PMDCam, ((float*) distortedData->data.ptr), height*width*sizeof(float));
				if (ret != PMD_OK)
				{
					pmdGetLastError (m_PMDCam, err, 128);
					std::cerr << "ERROR - PMDCamCube::AcquireImages():" << std::endl;
					std::cerr << "\t ... Could not get phase image data from camera" << std::endl;
					std::cerr << "\t ... '" << err << "'" << std::endl;
					return RET_FAILED;
				}

				// Calculate calibrated z values (in meter) based on 6 degree polynomial approximation
				for(unsigned int row=0; row<(unsigned int)height; row++)
				{
					for (unsigned int col=0; col<(unsigned int)width; col++)
					{
						zRaw = distortedData->data.fl[width*row + col];
						GetCalibratedZMatlab(col, row, zRaw, zCalibrated);
						((float*) (distortedData->data.ptr + row*widthStepOneChannel))[col] = zCalibrated;
					}	
				}

				//IplImage dummy;
				//IplImage *z = cvGetImage(distortedData, &dummy);
				//IplImage *image = cvCreateImage(cvGetSize(z), IPL_DEPTH_8U, 3);
				//ipa_Utils::ConvertToShowImage(z, image, 1);
				//cvNamedWindow("Z");
				//cvShowImage("Z", image);
				//cvWaitKey();
				

				// Undistort
				CvMat* undistortedData = cvCloneMat(distortedData);
				RemoveDistortion(distortedData, undistortedData);
				cvReleaseMat(&distortedData);

				// Calculate X and Y based on instrinsic rotation and translation
				for(unsigned int row=0; row<(unsigned int)height; row++)
				{
					for (unsigned int col=0; col<(unsigned int)width; col++)
					{
						zCalibrated = undistortedData->data.fl[width*row + col];

						GetCalibratedXYMatlab(col, row, zCalibrated, x, y);

						ptr = &((float*) (cartesianImageData + row*widthStepCartesianImage))[col*3];
						ptr[0] = x;
						ptr[1] = y;
						ptr[2] = zCalibrated;
						//CvScalar Val = cvScalar(x, y, z);
						//cvSet2D(*cartesianImage, row, col, Val);
					}
				}
				cvReleaseMat(&undistortedData);
			}
			else
			{
				std::cerr << "ERROR - PMDCamCube::AcquireImages: \n";
				std::cerr << "\t ... At least one of m_CoeffsA0 ... m_CoeffsA6 not initialized.\n";
				return RET_FAILED;
			}*/

		}
		else if(m_CalibrationMethod==MATLAB_NO_Z)
		{
			cv::Mat pmdData(height, 3*width, CV_32FC1);
			cv::Mat distortedData(height, width, CV_32FC1);

			//ret = pmdGetDistances(m_PMDCam, ((float*) distortedData->data.ptr), height*width*sizeof (float));
			ret = pmdGet3DCoordinates(m_PMDCam, (float*) pmdData.data, 3*height*width*sizeof (float));
			if (ret != PMD_OK)
			{
				pmdGetLastError (m_PMDCam, err, 128);
				std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
				std::cerr << "\t ... Could not get phase image data from camera" << std::endl;
				std::cerr << "\t ... '" << err << "'" << std::endl;
				return RET_FAILED;
			}

			// Left and right is mirrowed by PMD Cam
			for(unsigned int row=0; row<(unsigned int)height; row++)
			{
				f_ptr = pmdData.ptr<float>(row);
				f_ptr_dst = distortedData.ptr<float>(row);

				for (unsigned int col=0; col<(unsigned int)width; col++)
				{
					f_ptr_dst[width - col - 1] = f_ptr[col*3 + 2];
				}	
			}

			// Undistort
			cv::Mat undistortedData;

			assert (!m_undistortMapX.empty() && !m_undistortMapY.empty());
			cv::remap(distortedData, undistortedData, m_undistortMapX, m_undistortMapY, cv::INTER_LINEAR);

			// Calculate X and Y based on instrinsic rotation and translation
			for(unsigned int row=0; row<(unsigned int)height; row++)
			{
				zCalibratedPtr = undistortedData.ptr<float>(row);
				f_ptr = (float*) (cartesianImageData + row*widthStepCartesian);

				for (unsigned int col=0; col<(unsigned int)width; col++)
				{
					int colTimes3 = col*3;
					GetCalibratedXYMatlab(col, row, zCalibratedPtr[col], x, y);

					f_ptr[colTimes3] = x;
					f_ptr[colTimes3 + 1] = y;
					f_ptr[colTimes3 + 2] = zCalibratedPtr[col];
				}
			}
		}
		else if(m_CalibrationMethod==NATIVE)
		{
			cv::Mat pmdData(height, 3*width, CV_32FC1 );
			//ret = pmdGet3DCoordinates(m_PMDCam, ((float*) cartesianImageData), 3*height*width*sizeof (float));
			ret = pmdGet3DCoordinates(m_PMDCam, (float*) pmdData.data, 3*height*width*sizeof (float));
			if (ret != PMD_OK)
			{
				pmdGetLastError (m_PMDCam, err, 128);
				std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
				std::cerr << "\t ... Could not get cartesian image data from camera" << std::endl;
				std::cerr << "\t ... '" << err << "'" << std::endl;
				return RET_FAILED;
			}

			// Left and right is mirrowed by PMD Cam
			int withTimes3 = width*3;
			for(unsigned int row=0; row<(unsigned int)height; row++)
			{
				f_ptr = pmdData.ptr<float>(row);
				f_ptr_dst = (float*) (cartesianImageData + row*widthStepCartesian);

				for (unsigned int col=0; col<(unsigned int)width*3; col+=3)
				{
					x = f_ptr[col];
					y = f_ptr[col + 1];
					z = f_ptr[col + 2];
					f_ptr_dst[withTimes3 - col - 1] = z;
					f_ptr_dst[withTimes3 - col - 2] = y;
					f_ptr_dst[withTimes3 - col - 3] = x;
				}	
			}
		}
		else
		{
			std::cerr << "ERROR - PMDCamCube::AcquireImages:" << std::endl;
			std::cerr << "\t ... Calibration method unknown.\n";
			return RET_FAILED;
		}
	}
	return RET_OK;
}

unsigned long PMDCamCube::SaveParameters(const char* filename) 
{
	return RET_FUNCTION_NOT_IMPLEMENTED;
}

unsigned long PMDCamCube::GetCalibratedZMatlab(int u, int v, float zRaw, float& zCalibrated)
{
	double c[7] = {m_CoeffsA0.at<double>(v,u), m_CoeffsA1.at<double>(v,u), m_CoeffsA2.at<double>(v,u), 
		m_CoeffsA3.at<double>(v,u), m_CoeffsA4.at<double>(v,u), m_CoeffsA5.at<double>(v,u), m_CoeffsA6.at<double>(v,u)};
	double y = 0;
	ipa_Utils::EvaluatePolynomial((double) zRaw, 6, &c[0], &y);
	zCalibrated = (float) y;
	
	return RET_OK;
}


// u and v are assumed to be distorted coordinates
unsigned long PMDCamCube::GetCalibratedXYMatlab(int u, int v, float z, float& x, float& y)
{
	// Conversion form m to mm
	z *= 1000;

	// Use intrinsic camera parameters
	double fx, fy, cx, cy;
	fx = m_intrinsicMatrix.at<double>(0, 0);
	fy = m_intrinsicMatrix.at<double>(1, 1);

	cx = m_intrinsicMatrix.at<double>(0, 2);
	cy = m_intrinsicMatrix.at<double>(1, 2);

	// Fundamental equation: u = (fx*x)/z + cx
	if (fx == 0)
	{
		std::cerr << "ERROR - PMDCamCube::GetCalibratedXYZ:" << std::endl;
		std::cerr << "\t ... fx is 0.\n";
		return RET_FAILED;
	}
	x = (float) (z*(u-cx)/fx) ; 
	
	// Fundamental equation: v = (fy*y)/z + cy
	if (fy == 0)
	{
		std::cerr << "ERROR - PMDCamCube::GetCalibratedXYZ:" << std::endl;
		std::cerr << "\t ... fy is 0.\n";
		return RET_FAILED;
	}
	y = (float) (z*(v-cy)/fy); 

	// Conversion from mm to m
	x /= 1000;
	y /= 1000;

	return RET_OK;
}


unsigned long PMDCamCube::SetParameters()
{
	ipa_CameraSensors::t_cameraProperty cameraProperty;
	
// -----------------------------------------------------------------
// Set trigger
// -----------------------------------------------------------------
	if (m_RangeCameraParameters.m_CameraRole == MASTER)
	{
		int ret = 0;
		char err[128];

		ret = pmdSourceCommand (m_PMDCam, 0, 0, "SetTriggerMode Soft");
		if (ret != PMD_OK)
		{
			pmdGetLastError (0, err, 128);
			std::cerr << "ERROR - PMDCamCube::SetParameters" << std::endl;
			std::cerr << "\t ... Could not set trigger mode" << std::endl;
			std::cerr << "\t ... Assert, that 'camcube.W32.pap' plugin file supports triggering" << std::endl;
			std::cerr << "\t ... '" << err << "'" << std::endl;
		}
	}
	else if (m_RangeCameraParameters.m_CameraRole == SLAVE)
	{
		int ret = 0;
		char err[128];

		ret = pmdSourceCommand (m_PMDCam, 0, 0, "SetFrameTimeout 100000000000000000000");
		if (ret != PMD_OK)
		{
			pmdGetLastError (0, err, 128);
			std::cerr << "ERROR - PMDCamCube::SetParameters" << std::endl;
			std::cerr << "\t ... Could not set trigger timeout" << std::endl;
			std::cerr << "\t ... '" << err << "'" << std::endl;
		}

		ret = pmdSourceCommand (m_PMDCam, 0, 0, "SetTriggerMode Hard");
		if (ret != PMD_OK)
		{
			pmdGetLastError (0, err, 128);
			std::cerr << "ERROR - PMDCamCube::SetParameters" << std::endl;
			std::cerr << "\t ... Could not set trigger mode" << std::endl;
			std::cerr << "\t ... Assert, that 'camcube.W32.pap' plugin file supports triggering" << std::endl;
			std::cerr << "\t ... '" << err << "'" << std::endl;
		}
	}

// -----------------------------------------------------------------
// Set integration time
// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_INTEGRATION_TIME;
	std::string sIntegrationTime = "";
	m_RangeCameraParameters.m_IntegrationTime.seekg(0); // Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_IntegrationTime >> sIntegrationTime;
	m_RangeCameraParameters.m_IntegrationTime.clear();
	if (sIntegrationTime == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sIntegrationTime == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_RangeCameraParameters.m_IntegrationTime.seekg(0); // Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_IntegrationTime >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - PMDCamCube::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set integration time" << std::endl;
	}

// -----------------------------------------------------------------
// Set modulation frequency
// -----------------------------------------------------------------
/*	cameraProperty.propertyID = ipa_CameraSensors::PROP_MODULATION_FREQUENCY;
	std::string sModulationFrequency = "";
	m_RangeCameraParameters.m_ModulationFrequency.seekg(0); // Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_ModulationFrequency >> sModulationFrequency;
	if (sModulationFrequency == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sModulationFrequency == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_UNSIGNED | ipa_CameraSensors::TYPE_LONG);
		m_RangeCameraParameters.m_ModulationFrequency.seekg(0); // Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_ModulationFrequency >> cameraProperty.u_longData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - PMDCamCube::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set modulation frequency" << std::endl;
	}*/

// -----------------------------------------------------------------
// Set distance offset
// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_DISTANCE_OFFSET;
	std::string sDistanceOffset = "";
	m_RangeCameraParameters.m_DistanceOffset.seekg(0); // Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_DistanceOffset >> sDistanceOffset;
	if (sDistanceOffset == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sDistanceOffset == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_STRING);
		m_RangeCameraParameters.m_DistanceOffset.seekg(0); // Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_DistanceOffset >> cameraProperty.stringData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - PMDCamCube::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set distance offset" << std::endl;
	}

// -----------------------------------------------------------------
// Set ROI
// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_ROI;

	std::string sROI = "";
	m_RangeCameraParameters.m_ROI.seekg(0); // Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_ROI >> sROI;
	if (sROI == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sROI == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_STRING);
		m_RangeCameraParameters.m_DistanceOffset.seekg(0); // Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_ROI >> cameraProperty.stringData;
		m_RangeCameraParameters.m_ROI >> cameraProperty.stringData;
		m_RangeCameraParameters.m_ROI >> cameraProperty.stringData;
		m_RangeCameraParameters.m_ROI >> cameraProperty.stringData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - PMDCamCube::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set ROI" << std::endl;
	}

// -----------------------------------------------------------------
// Set exposure mode
// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_EXPOSURE_TIME;
	std::string sExposureMode = "";
	m_RangeCameraParameters.m_ExposureMode.seekg(0); // Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_ExposureMode >> sExposureMode;
	if (sExposureMode == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sExposureMode == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_STRING);
		m_RangeCameraParameters.m_ExposureMode.seekg(0); // Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_ExposureMode >> cameraProperty.stringData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - PMDCamCube::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set exposure mode" << std::endl;
	}

// -----------------------------------------------------------------
// Set calibration method
// -----------------------------------------------------------------
	cameraProperty.propertyID = ipa_CameraSensors::PROP_LENS_CALIBRATION;
	std::string sLensCalibration = "";
	m_RangeCameraParameters.m_LensCalibration.seekg(0); // Set Pointer to position 0 within stringstream
	m_RangeCameraParameters.m_LensCalibration >> sLensCalibration;
	m_RangeCameraParameters.m_LensCalibration.clear();
	if (sLensCalibration == "AUTO")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_AUTO;
	}
	else if (sLensCalibration == "DEFAULT")
	{
		cameraProperty.propertyType = ipa_CameraSensors::TYPE_SPECIAL;
		cameraProperty.specialValue = VALUE_DEFAULT;
	}
	else
	{
		cameraProperty.propertyType = (ipa_CameraSensors::TYPE_STRING);
		m_RangeCameraParameters.m_LensCalibration.seekg(0); // Set Pointer to position 0 within stringstream
		m_RangeCameraParameters.m_LensCalibration >> cameraProperty.stringData;
	}

	if (SetProperty(&cameraProperty) & ipa_CameraSensors::RET_FAILED)
	{
		std::cout << "WARNING - PMDCamCube::SetParameters:" << std::endl;
		std::cout << "\t ... Could not set calibration mode" << std::endl;
	}

	return RET_OK;
}

unsigned long PMDCamCube::LoadParameters(const char* filename, int cameraIndex)
{
	// Load SwissRanger parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - PMDCamCube::LoadParameters:" << std::endl;
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
			// Tag element "Swissranger3000" of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_SR31 = NULL;
			std::stringstream ss;
			ss << "PMDCamCube_" << cameraIndex;
			p_xmlElement_Root_SR31 = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_SR31 )
			{

//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->Role
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "Role" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Role'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_RangeCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_RangeCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}

				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Role'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->Serial
//************************************************************************************
				// Subtag element "AmplitudeThreshold" of XML Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "Serial" );
				if ( p_xmlElement_Child )
				{
					m_Serial = "";
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "low", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'low' of tag 'Serial'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					
					m_Serial = tempString;
					m_Serial += ".";

					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "high", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ...  Can't find attribute 'high' of tag 'Serial'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					
					m_Serial += tempString;
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Serial'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->DistanceOffset
//************************************************************************************
				// Subtag element "AmplitudeThreshold" of XML Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "DistanceOffset" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'DistanceOffset'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_DistanceOffset.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_DistanceOffset.clear();		// Reset flags
						m_RangeCameraParameters.m_DistanceOffset << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'DistanceOffset'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->ROI
//************************************************************************************
				// Subtag element "AmplitudeThreshold" of XML Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "ROI" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "left", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'left' of tag 'ROI'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_ROI.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_ROI.clear();		// Reset flags
						m_RangeCameraParameters.m_ROI << tempString;
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "top", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'top' of tag 'ROI'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_ROI << " ";
						m_RangeCameraParameters.m_ROI << tempString;
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "width", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'width' of tag 'ROI'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_ROI << " ";
						m_RangeCameraParameters.m_ROI << tempString;
					}
					if ( p_xmlElement_Child->QueryValueAttribute( "height", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'height' of tag 'ROI'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_ROI << " ";
						m_RangeCameraParameters.m_ROI << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'ROI'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->ExposureMode
//************************************************************************************
				// Subtag element "AmplitudeThreshold" of XML Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "ExposureMode" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "mode", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'mode' of tag 'ExposureMode'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_ExposureMode.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_ExposureMode.clear();		// Reset flags
						m_RangeCameraParameters.m_ExposureMode << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'ExposureMode'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->IntegrationTime
//************************************************************************************
				// Subtag element "AmplitudeThreshold" of XML Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "IntegrationTime" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'IntegrationTime'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_RangeCameraParameters.m_IntegrationTime.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_IntegrationTime.clear();		// Reset flags
						m_RangeCameraParameters.m_IntegrationTime << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'IntegrationTime'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}


//************************************************************************************
//	BEGIN LibCameraSensors->PMDCamCube->CalibrationMethod
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_SR31->FirstChildElement( "CalibrationMethod" );
				std::string tempString;
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "name", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'name' of tag 'CalibrationMethod'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "MATLAB") 
					{
						m_CalibrationMethod = MATLAB;
						m_RangeCameraParameters.m_LensCalibration.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_LensCalibration.clear();		// Reset flags
						m_RangeCameraParameters.m_LensCalibration << "OFF";
					}
					else if (tempString == "MATLAB_NO_Z")
					{
						m_CalibrationMethod = MATLAB_NO_Z;
						m_RangeCameraParameters.m_LensCalibration.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_LensCalibration.clear();		// Reset flags
						m_RangeCameraParameters.m_LensCalibration << "ON";
					}
					else if (tempString == "NATIVE") 
					{
						m_CalibrationMethod = NATIVE;
						m_RangeCameraParameters.m_LensCalibration.str( " " );	// Clear stringstream
						m_RangeCameraParameters.m_LensCalibration.clear();		// Reset flags
						m_RangeCameraParameters.m_LensCalibration << "ON";
					}
					else
					{
						std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CalibrationMethod'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}
//************************************************************************************
//	END LibCameraSensors->PMDCamCube
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - PMDCamCube::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "INFO - PMDCamCube::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml calibration file: Done.\n";

	

	return RET_OK;
}
