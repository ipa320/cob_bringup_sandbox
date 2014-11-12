/****************************************************************
*
* Copyright (c) 2010
*
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Project name: care-o-bot
* ROS stack name: cob_driver
* ROS package name: cob_camera_sensors
* Description: Abstract interface for color cameras.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: May 2011
* ToDo:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

/// @file AbstractColorCamera.h
/// Abstract interface for color cameras.
/// @author Jan Fischer
/// @date May 2008.

#ifndef __IPA_OPENCVCAMERA_H__
#define __IPA_OPENCVCAMERA_H__

#ifdef __LINUX__
	#include "cob_camera_sensors/AbstractColorCamera.h"
#else
	#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#endif

#include <opencv2/core/core.hpp>
#include <set>



namespace ipa_CameraSensors {

#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
	#include <assert.h>

	#include <getopt.h>             /* getopt_long() */

	#include <fcntl.h>              /* low-level i/o */
	#include <unistd.h>
	#include <errno.h>
	#include <malloc.h>
	#include <sys/stat.h>
	#include <sys/types.h>
	#include <sys/time.h>
	#include <sys/mman.h>
	#include <sys/ioctl.h>

	#include <asm/types.h>          /* for videodev2.h */
	#include <iostream>
	#include <linux/videodev2.h>


	struct buffer 
	{
		void* start;
		size_t length;
	};
#endif

class __DLL_LIBCAMERASENSORS__ OpenCVCamera : public AbstractColorCamera 
{
	public:

		/// Constructor
		OpenCVCamera();
		
		/// Destructor
		~OpenCVCamera();

		/// Initializes the color camera.
		/// Camera specific constants may be set within the configuration file <I>cameraSensorsIni.xml</I>.
		/// The function has to set the member variable <code>m_initialized</code>.
		/// @param directory Path to the configuration file directory.
		/// @param cameraIndex It is possible to have several cameras of the same type on the system.
		///	       One may us the camera index to apply different configuration files to each of them
		/// @return Return code.
		unsigned long Init(std::string directory, int cameraIndex = 0);
	
		/// Returns true, when <code>Init()</code> has been called on the camera.
		/// @return Camera initialized or not.
		bool isInitialized() {return m_initialized;}

		/// Returns true, when <code>Open()</code> has been called on the camera.
		/// @return Camera opened or not.
		bool isOpen() {return m_open;}

		/// Opens the camera device.
		/// All camera specific parameters for opening the camera should have been set within the <code>Init</code>
		/// function.
		/// @return Return code.
		unsigned long Open();

		/// Close camera device.
		/// @return Return code.
		unsigned long Close(); //Save intrinsic params back to File
		
		/// Retrieves image data from the color camera.
		/// @param colorImageData An array to be filled with image data
		/// @param getLatestFrame True, when the latest picture has to be returned. Otherwise, the next picture
		///						  following the last call to <code>getLatestFrame</code> is returned.
		/// @return Return code
		unsigned long GetColorImage(char* colorImageData, bool getLatestFrame=true) {return RET_FAILED;}

		/// Retrieves an image from the camera.
		/// <code>cv::Mat</code> object is initialized on demand.
		/// @param colorImage The image that has been acquired by the camera.
		/// @param getLatestFrame If true, the camera acquires a new frame and returns it.
		///						  Otherwise, the next frame following the last returned frame
		///						  is returned from the internal camera buffer.
		/// @throw IPA_Exception Throws an exception, if camera access failed
		unsigned long GetColorImage(cv::Mat* colorImage, bool getLatestFrame=true);

		/// Returns the camera type.
		/// @return The camera type
		t_cameraType GetCameraType() { return m_CameraType; }
	
		/// Function to set properties of the camera sensor.
		/// @param propertyID The ID of the property.
		/// @param cameraProperty The value of the property.
		/// @return Return code.
		unsigned long SetProperty(t_cameraProperty* cameraProperty) {return RET_FAILED;};

		/// Function to set property defaults of the camera sensor.
		/// @return Return code.
		unsigned long SetPropertyDefaults() {return RET_FAILED;};

		/// Function to get properties of the camera sensor.
		/// @param propertyID The ID of the property.
		/// @param cameraProperty The value of the property.
		/// @return Return code.
		unsigned long GetProperty(t_cameraProperty* cameraProperty);

		/// Displays camera information on standard output.
		/// Information includes available parameters, color and camera formats.
		/// @return Return code.
		unsigned long PrintCameraInformation() {return RET_FAILED;};

		/// Saves all parameters on hard disk.
		/// @param filename The filename of the storage.
		/// @return Return code.
		unsigned long SaveParameters(const char* filename) {return RET_FAILED;};

		/// Unit Test for the camera interface.
		/// Tests each of the single interface functions and displays the output on
		/// standard out.
		/// @param filename Path to the camera initialization xml file.
		/// @return Return code.
		unsigned long TestCamera(const char* filename);

		/// Returns the number of images in the directory
		/// @return The number of images in the directory
		int GetNumberOfImages() {return std::numeric_limits<int>::max();};

		/// Function specific to virtual camera.
		/// Resets the image directory read from the configuration file.
		/// @param path The camera path
		/// @return Return code
		unsigned long SetPathToImages(std::string path) {return RET_OK;};

	protected:
		
		static std::set<int> m_initializedCameraIndices;	///< contains all camera indices which were already initialized to prevent double usage of the same device
		bool m_initialized; ///< True, when the camera has sucessfully been initialized.
		bool m_open;		///< True, when the camera has sucessfully been opend.

		cv::VideoCapture* m_cameraDevice;	///< Pointer to the OpenCV video camera device

		int m_cameraIndex;	///< index of the used camera (0=standard camera, >0 if multiple cameras are attached to the computer)

		t_ColorCameraParameters m_ColorCameraParameters; ///< Storage for xml configuration file data

		t_cameraType m_CameraType; ///< Camera Type

		unsigned int m_BufferSize; ///< Number of images, the camera buffers internally

#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
		void errno_exit(const char* s);

		int xioctl (int fd, int request, void* arg);

		inline int clamp(int value) { return std::min(std::max(value, 0), 255); };
		
		int frame_copy(const void *p, cv::Mat& img);

		int read_frame(cv::Mat& img);

		void stop_capturing(void);

		void start_capturing(void);

		void uninit_device(void);

		void init_mmap(void);

		void init_device(void);

		void close_device(void);

		void open_device();


		struct buffer* m_buffers;
		unsigned int m_n_buffers;
		int m_fd;
		std::string m_device_name;

#endif

	private:

		/// Loads all camera specific parameters from the xml configuration file and saves them in t_ColorCameraParameters.
		/// This function is internally called by Init to load the parameters from the xml configuration file.
		/// @param filename The path to the configuration file.
		/// @return Return code.
		unsigned long LoadParameters(const char* filename, int cameraIndex);

		/// Sets the loaded parameters.
		/// @return Return code.
		unsigned long SetParameters() {return RET_OK;};

};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_OpenCVCamera();


} // end namespace
#endif // __IPA_OPENCVCAMERA_H__
