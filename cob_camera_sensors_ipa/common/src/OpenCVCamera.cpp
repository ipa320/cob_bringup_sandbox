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
* Description:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: Mai 2011
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
#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
#include "cob_camera_sensors_ipa/OpenCVCamera.h"

#include "tinyxml/tinyxml.h"
#include <iostream>
#else
#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/OpenCVCamera.h"

#endif


using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr ipa_CameraSensors::CreateColorCamera_OpenCVCamera()
{
	return AbstractColorCameraPtr(new OpenCVCamera());
}


std::set<int> OpenCVCamera::m_initializedCameraIndices;


OpenCVCamera::OpenCVCamera()
{
	m_initialized = false;
	m_open = false;
	m_BufferSize = 1;
	m_cameraDevice = 0;

#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
	m_buffers = 0;
	m_n_buffers = 0;
	m_fd = -1;
	m_device_name = "/dev/video0";
#endif
}


OpenCVCamera::~OpenCVCamera()
{
	m_initializedCameraIndices.erase(m_cameraIndex);

	if (m_cameraDevice != 0)
	{
		delete m_cameraDevice;
		m_cameraDevice = 0;
	}
}


unsigned long OpenCVCamera::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	// load parameters
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - OpenCVCamera::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return RET_FAILED;
	}

	m_cameraIndex = 0;//cameraIndex; //todo: read this parameter from the ini file

	// do not double initialize the same device
	if (m_initializedCameraIndices.find(m_cameraIndex) != m_initializedCameraIndices.end())
	{
		std::cerr << "ERROR - OpenCVCamera::Init:" << std::endl;
		std::cerr << "\t ... The desired camera device has already been initialized by another instance of OpenCVCamera." << std::endl;
		return RET_FAILED;
	}
	m_initializedCameraIndices.insert(m_cameraIndex);

	// set camera type
	m_CameraType = ipa_CameraSensors::CAM_OPENCVCAMERA;

	// Set init flag
	m_initialized = true;

	return RET_OK;
}


unsigned long OpenCVCamera::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	std::cout << "INFO - OpenCVCamera::Open():" << std::endl;
	std::cout << "\t ... Opening camera device" << std::endl;
	std::cout << "\t ... This may take some seconds" << std::endl;

	// open camera device
#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
	// directly access v4l driver
	open_device();
	init_device();
	start_capturing();

	// get some initial images to start with good images later
	int width, height;
	std::stringstream ss;
	ss << m_ColorCameraParameters.m_ImageWidth.str();
	ss >> width;
	ss.str("");
	ss.clear();
	ss << m_ColorCameraParameters.m_ImageHeight.str();
	ss >> height;
	cv::Mat dummyImg(height, width, CV_8UC3);
	for (int i=0; i<10; i++)
		GetColorImage(&dummyImg);
#else
	// use the standard opencv driver
	m_cameraDevice = new cv::VideoCapture(m_cameraIndex);
	if (!m_cameraDevice->isOpened())
	{
		std::cerr << "ERROR - OpenCVCamera::Open:" << std::endl;
		std::cerr << "\t ... Opening the video camera device failed." << std::endl;
		return RET_FAILED;
	}
	
	// set camera parameters
	int width, height;
	m_ColorCameraParameters.m_ImageWidth >> width;
	m_ColorCameraParameters.m_ImageWidth.seekg(std::ios_base::beg);
	m_ColorCameraParameters.m_ImageHeight >> height;
	m_ColorCameraParameters.m_ImageHeight.seekg(std::ios_base::beg);
	m_cameraDevice->set(CV_CAP_PROP_FRAME_WIDTH, width);
	m_cameraDevice->set(CV_CAP_PROP_FRAME_HEIGHT, height);
#endif

	std::cout << "*************************************************" << std::endl;
	std::cout << "OpenCVCamera::Open: OpenCV camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long OpenCVCamera::Close()
{
	m_open = false;

#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
	stop_capturing ();
	uninit_device ();
	close_device ();
#else
	if (m_cameraDevice != 0)
	{
		delete m_cameraDevice;
		m_cameraDevice = 0;
	}
#endif

	return RET_OK;
}


unsigned long OpenCVCamera::GetColorImage(cv::Mat* colorImage, bool getLatestFrame)
{
#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
	for (;;)
	{
		fd_set fds;
		struct timeval tv;
		int r;

		FD_ZERO (&fds);
		FD_SET (m_fd, &fds);

		/* Timeout. */
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		r = select (m_fd + 1, &fds, NULL, NULL, &tv);

		if (-1 == r)
		{
			if (EINTR == errno)
				continue;

			errno_exit ("select");
		}

		if (0 == r)
		{
			fprintf (stderr, "select timeout\n");
			exit (EXIT_FAILURE);
		}

		if (read_frame(*colorImage))
			break;
		
		/* EAGAIN - continue select loop. */
	}
#else
		*m_cameraDevice >> *colorImage;
#endif

	return RET_OK;
}


unsigned long OpenCVCamera::GetProperty(t_cameraProperty* cameraProperty)
{
	int ret = 0;

#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:	
			struct v4l2_format fmt;
			memset (&(fmt), 0, sizeof(fmt));
			fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

			if (-1 == xioctl (m_fd, VIDIOC_G_FMT, &fmt))
				errno_exit ("VIDIOC_S_FMT");

			cameraProperty->cameraResolution.xResolution = fmt.fmt.pix.width;
			cameraProperty->cameraResolution.yResolution = fmt.fmt.pix.height;
			break;
		case PROP_BRIGHTNESS:	
		case PROP_WHITE_BALANCE_U:	
		case PROP_HUE:	
		case PROP_SATURATION:	
		case PROP_GAMMA:	
		case PROP_EXPOSURE_TIME:	
		case PROP_GAIN:	
		case PROP_OPTICAL_FILTER:	
		case PROP_FRAME_RATE:	
		case PROP_REGISTER:	
		case PROP_TIMEOUT:	
		default: 				
			std::cout << "OpenCVCamera::GetProperty: Property " << cameraProperty->propertyID << " unspecified.";
			ret = -1; 
			break;
	}
#else
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:	
			cameraProperty->cameraResolution.xResolution = m_cameraDevice->get(CV_CAP_PROP_FRAME_WIDTH);
			cameraProperty->cameraResolution.yResolution = m_cameraDevice->get(CV_CAP_PROP_FRAME_HEIGHT);
			break;
		case PROP_BRIGHTNESS:	
		case PROP_WHITE_BALANCE_U:	
		case PROP_HUE:	
		case PROP_SATURATION:	
		case PROP_GAMMA:	
		case PROP_EXPOSURE_TIME:	
		case PROP_GAIN:	
		case PROP_OPTICAL_FILTER:	
		case PROP_FRAME_RATE:	
		case PROP_REGISTER:	
		case PROP_TIMEOUT:	
		default: 				
			std::cout << "OpenCVCamera::GetProperty: Property " << cameraProperty->propertyID << " unspecified.";
			ret = -1; 
			break;
	}
#endif

	if (ret < 0)
	{
		return RET_FAILED;
	}
	else
	{
		return RET_OK;
	}
}


unsigned long OpenCVCamera::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}
	return RET_OK;
}


unsigned long OpenCVCamera::LoadParameters(const char* filename, int cameraIndex)
{
	// Load OpenCVCamera parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - OpenCVCamera::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - OpenCVCamera::LoadParameters:" << std::endl;
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
//	BEGIN LibCameraSensors->OpenCVCamera
//************************************************************************************
			// Tag element "OpenCVCamera of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_OCVC = NULL;
			std::stringstream ss;
			ss << "OpenCVCamera_" << cameraIndex;
			p_xmlElement_Root_OCVC = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_OCVC )
			{
				
//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->Resolution
//************************************************************************************
				// Subtag element "XSize" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OCVC->FirstChildElement( "Resolution" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("width", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - OpenCVCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'width' of tag 'Resolution'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ImageWidth.str( " " );	// Clear stringstream
						m_ColorCameraParameters.m_ImageWidth.clear();		// Reset flags
						m_ColorCameraParameters.m_ImageWidth << tempString;
					}
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("height", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - OpenCVCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'height' of tag 'Resolution'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ImageHeight.str( " " );	// Clear stringstream
						m_ColorCameraParameters.m_ImageHeight.clear();		// Reset flags
						m_ColorCameraParameters.m_ImageHeight << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - OpenCVCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Resolution'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->OpenCVCamera
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - OpenCVCamera::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - OpenCVCamera::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "\t [OK] Parsing xml calibration file\n";

	return RET_OK;
}


#if defined __LINUX__ && defined __USE_FAST_V4L_DRIVER__
void OpenCVCamera::errno_exit(const char* s)
{
	fprintf (stderr, "%s error %d, %s\n", s, errno, strerror (errno));
	exit (EXIT_FAILURE);
}

int OpenCVCamera::xioctl (int m_fd, int request, void* arg)
{
		int r;

		do r = ioctl (m_fd, request, arg);
		while (-1 == r && EINTR == errno);

		return r;
}

// copies the frame from the capture buffer to a cv::Mat and converts YUYV to BGR
int OpenCVCamera::frame_copy(const void *p, cv::Mat& img)
{
	// convert V4L2_PIX_FMT_YUYV to BGR24
	int Y0, Y1, Cb, Cr;            /* gamma pre-corrected input [0;255] */
	char result[6];			/* output [0;255] */
	double r0,r1,g0,g1,b0,b1;             /* temporaries */
	double y0,y1, pb, pr;
	uchar* ptr  = img.ptr();

	for (int i=0; i<(img.rows*img.cols/2); i++)
	{
		int packed_value = *((int*)p+i);

		Y0 = (char)(packed_value & 0xFF);
		Cb = (char)((packed_value >> 8) & 0xFF);
		Y1 = (char)((packed_value >> 16) & 0xFF);
		Cr = (char)((packed_value >> 24) & 0xFF);

		// Strip sign values after shift (i.e. unsigned shift)
		Y0 = Y0 & 0xFF;
		Cb = Cb & 0xFF;
		Y1 = Y1 & 0xFF;
		Cr = Cr & 0xFF;

		y0 = (255 / 219.0) * (Y0 - 16);
		y1 = (255 / 219.0) * (Y1 - 16);
		pb = (255 / 224.0) * (Cb - 128);
		pr = (255 / 224.0) * (Cr - 128);

		// Generate first pixel
		r0 = 1.0 * y0 + 0     * pb + 1.402 * pr;
		g0 = 1.0 * y0 - 0.344 * pb - 0.714 * pr;
		b0 = 1.0 * y0 + 1.772 * pb + 0     * pr;

		// Generate next pixel - must reuse pb & pr as 4:2:2
		r1 = 1.0 * y1 + 0     * pb + 1.402 * pr;
		g1 = 1.0 * y1 - 0.344 * pb - 0.714 * pr;
		b1 = 1.0 * y1 + 1.772 * pb + 0     * pr;

		// keep values in [0,255]
		result[0] = clamp(b0);
		result[1] = clamp(g0);
		result[2] = clamp(r0);
		result[3] = clamp(b1);
		result[4] = clamp(g1);
		result[5] = clamp(r1);

		// write to cv::Mat
		memcpy(ptr, result, 6);
		ptr += 6;
	}
	return RET_OK;
}

// reads a frame from the m_buffers
int OpenCVCamera::read_frame(cv::Mat& img)
{
	struct v4l2_buffer buf;

	memset(&(buf), 0, sizeof(buf));

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(m_fd, VIDIOC_DQBUF, &buf))
	{
		switch (errno)
		{
		case EAGAIN:
			return 0;
		case EIO:
			// Could ignore EIO, see spec.
			// fall through
		default:
			errno_exit ("VIDIOC_DQBUF");
		}
	}

	assert(buf.index < m_n_buffers);
	
	// copy image
	if (m_buffers[buf.index].start)
		frame_copy(m_buffers[buf.index].start, img);

	if (-1 == xioctl (m_fd, VIDIOC_QBUF, &buf))
		errno_exit ("VIDIOC_QBUF");

	return RET_OK;
}

void OpenCVCamera::stop_capturing(void)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (m_fd, VIDIOC_STREAMOFF, &type))
		errno_exit ("VIDIOC_STREAMOFF");
}

void OpenCVCamera::start_capturing(void)
{
	enum v4l2_buf_type type;

	for (unsigned int i = 0; i < m_n_buffers; ++i)
	{
		struct v4l2_buffer buf;

		memset (&(buf), 0, sizeof(buf));

		buf.type    = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory  = V4L2_MEMORY_MMAP;
		buf.index   = i;

		if (-1 == xioctl (m_fd, VIDIOC_QBUF, &buf))
			errno_exit ("VIDIOC_QBUF");
	}
				
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (-1 == xioctl (m_fd, VIDIOC_STREAMON, &type))
		errno_exit ("VIDIOC_STREAMON");
}

void OpenCVCamera::uninit_device(void)
{
	for (unsigned int i=0; i<m_n_buffers; ++i)
		if (-1 == munmap (m_buffers[i].start, m_buffers[i].length))
			errno_exit ("munmap");
	free(m_buffers);
}

void OpenCVCamera::init_mmap(void)
{
	struct v4l2_requestbuffers req;

	memset (&(req), 0, sizeof(req));

	req.count  = 4;
	req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(m_fd, VIDIOC_REQBUFS, &req))
	{
		if (EINVAL == errno)
		{
			fprintf (stderr, "%s does not support memory mapping\n", m_device_name.c_str());
			exit (EXIT_FAILURE);
		}
		else
		{
			errno_exit ("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2)
	{
		fprintf (stderr, "Insufficient buffer memory on %s\n", m_device_name.c_str());
		exit (EXIT_FAILURE);
	}

	m_buffers = (buffer*)calloc(req.count, sizeof(*m_buffers));

	if (!m_buffers)
	{
		fprintf (stderr, "Out of memory\n");
		exit (EXIT_FAILURE);
	}

	for (m_n_buffers=0; m_n_buffers<req.count; ++m_n_buffers)
	{
		struct v4l2_buffer buf;

		memset (&(buf), 0, sizeof(buf));

		buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index  = m_n_buffers;

		if (-1 == xioctl (m_fd, VIDIOC_QUERYBUF, &buf))
			errno_exit ("VIDIOC_QUERYBUF");

		m_buffers[m_n_buffers].length = buf.length;
		m_buffers[m_n_buffers].start = mmap (NULL /* start anywhere */,	buf.length,	PROT_READ | PROT_WRITE /* required */, MAP_SHARED /* recommended */, m_fd, buf.m.offset);

		if (MAP_FAILED == m_buffers[m_n_buffers].start)
			errno_exit ("mmap");
	}
}

void OpenCVCamera::init_device(void)
{
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;

	if (-1 == xioctl (m_fd, VIDIOC_QUERYCAP, &cap))
	{
		if (EINVAL == errno)
		{
			fprintf (stderr, "%s is no V4L2 device\n", m_device_name.c_str());
			exit (EXIT_FAILURE);
		}
		else
		{
			errno_exit ("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
	{
		fprintf (stderr, "%s is no video capture device\n", m_device_name.c_str());
		exit (EXIT_FAILURE);
	}

	if (!(cap.capabilities & V4L2_CAP_STREAMING))
	{
		fprintf (stderr, "%s does not support streaming i/o\n", m_device_name.c_str());
		exit (EXIT_FAILURE);
	}

	/* Select video input, video standard and tune here. */
	memset (&(cropcap), 0, sizeof(cropcap));
	
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl (m_fd, VIDIOC_CROPCAP, &cropcap))
	{
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl (m_fd, VIDIOC_S_CROP, &crop))
		{
			switch (errno)
			{
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	}
	else
	{
		/* Errors ignored. */
	}

	memset (&(fmt), 0, sizeof(fmt));

	int width, height;
	m_ColorCameraParameters.m_ImageWidth >> width;
	m_ColorCameraParameters.m_ImageWidth.seekg(ios_base::beg);
	m_ColorCameraParameters.m_ImageHeight >> height;
	m_ColorCameraParameters.m_ImageHeight.seekg(ios_base::beg);

	fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width       = width; 
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field       = V4L2_FIELD_ANY;

	if (-1 == xioctl (m_fd, VIDIOC_S_FMT, &fmt))
		errno_exit ("VIDIOC_S_FMT");

	/* Note VIDIOC_S_FMT may change width and height. */

	init_mmap ();
}

void OpenCVCamera::close_device(void)
{
	if (-1 == close (m_fd))
		errno_exit ("close");
	m_fd = -1;
}

void OpenCVCamera::open_device()
{
	struct stat st; 

	if (-1 == stat (m_device_name.c_str(), &st))
	{
		fprintf (stderr, "Cannot identify '%s': %d, %s\n", m_device_name.c_str(), errno, strerror (errno));
		exit (EXIT_FAILURE);
	}

	if (!S_ISCHR (st.st_mode))
	{
		fprintf (stderr, "%s is no device\n", m_device_name.c_str());
		exit (EXIT_FAILURE);
	}

	m_fd = open (m_device_name.c_str(), O_RDWR /* required */ | O_NONBLOCK, 0);

	if (-1 == m_fd)
	{
		fprintf (stderr, "Cannot open '%s': %d, %s\n", m_device_name.c_str(), errno, strerror (errno));
		exit (EXIT_FAILURE);
	}
}

#endif
