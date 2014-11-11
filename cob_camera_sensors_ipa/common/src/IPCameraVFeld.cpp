#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/IPCameraVFeld.h"

	#include "cob_vision_utils/memJpegDecoder.h"
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/IPCameraVFeld.h"

	#include "cob_object_perception_intern/windows/src/extern/MemJpegDecoder/memJpegDecoder.h"
#endif



using namespace ipa_CameraSensors;

#ifndef __LINUX__

CameraAxisJPEGStreamQueue::CameraAxisJPEGStreamQueue()
{
	m_first = (CameraAxisJPEGStreamQueueEntry*) malloc(sizeof(CameraAxisJPEGStreamQueueEntry));
	m_first->data = 0;
	m_first->next = 0;
	m_last = 0;
}

CameraAxisJPEGStreamQueue::~CameraAxisJPEGStreamQueue()
{
	CameraAxisJPEGStreamQueueEntry *cur, *del;

	cur = m_first;
	while (cur)
	{
		del = cur;
		cur = cur->next;
		free(del->data);
		free(del);
	}
}

unsigned long CameraAxisJPEGStreamQueue::Add(char *data, int dataLen)
{
	// Mutex is unlocked automatically when scoped_lock is destructed
	boost::mutex::scoped_lock lock(m_StreamQueueMutex);
	
	if (data && (dataLen != 0))
	{
		if (m_first->data)
		{
			free(m_first->data);
			m_first->data = 0;
		}
		m_first->data = (char*) malloc(dataLen);
		memcpy(m_first->data, data, dataLen);
		m_first->dataLen = dataLen;
		// Notify waiting threads
		m_ConditionPictureAvailable.notify_one();
	}
	else
	{
		std::cerr << "ERROR - CameraAxisJPEGStreamQueue::Add:" << std::endl;
		std::cerr << "\t ... data or dataLen is 0.\n";
		return RET_FAILED;
	}

	return RET_OK;
}

unsigned long CameraAxisJPEGStreamQueue::Get(char** data, int& dataLen)
{
	unsigned long return_value = RET_FAILED;

	// Mutex is unlocked automatically when scoped_lock is destructed
	// (when scoped_lock is out of scope)
	boost::mutex::scoped_lock lock(m_StreamQueueMutex);
	// wait  will atomically add the thread to the set of threads waiting
	// on the condition variable, and unlock the mutex. When the thread is woken,
	// the mutex will be locked again before the call to wait returns.
	m_ConditionPictureAvailable.wait(lock);

	if (m_first->data && (m_first->dataLen))
	{
		*data = (char*) malloc(m_first->dataLen);
		memcpy(*data, m_first->data, m_first->dataLen);
		dataLen = m_first->dataLen;
		
		free(m_first->data);
		m_first->data = 0;

		//std::cout << "CameraAxisJPEGStreamQueue::Get: dataLen is '" << dataLen << "' .\n";
		//std::cout << "CameraAxisJPEGStreamQueue::Get: data is '" << *data << "' .\n";
		return_value = RET_OK;
	}
	else
	{
		return_value = RET_FAILED;
	}
	
	return return_value;
}

IPCamera::IPCamera()
{
	m_initialized = false;
	m_open = false;
	m_bProcessingCriticalSection = false;

	m_JpegDecoder = 0;
	m_GetHTTPDataThread = 0;
	m_hInternet = 0;
	m_dynamicBufferLen = 0;
	m_hURL = 0;
	m_pJPEGStreamQueue = 0;
}

IPCamera::~IPCamera()
{
	if (m_open)
	{
		Close();
	}
}

unsigned long IPCamera::Init(int bufferSize, const char* url, t_mode mode)
{
	if (m_initialized)
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}
	
	m_bufferSize = bufferSize;
	m_pJPEGStreamQueue = new CameraAxisJPEGStreamQueue();
	
	m_url = (char*) malloc(strlen(url)+1);
	strcpy(m_url, url);
	m_mode = mode;
	m_JPEGStart = -1;

	m_JpegDecoder = new memJpegDecoder();

	m_hInternet = InternetOpen("HTTPClient",
		INTERNET_OPEN_TYPE_PRECONFIG, NULL, NULL, 0);
	if (!m_hInternet) 
	{
		std::cerr << "IPCamera::Init:" << std::endl;
		std::cerr << "\t ... Unable to setup HTTP client.\n";
		return RET_FAILED;
	}

	m_initialized = true;
	return RET_OK;
}

unsigned long IPCamera::Open()
{
	if (!m_initialized)
	{
		std::cerr << "ERROR - IPCamera::Open:" << std::endl;
		std::cerr << "\t ... Camera not initialized.";
		return RET_FAILED;
	}

	if (m_open)
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	// Setup buffer for HTTP data
	m_pDynamicBuffer = (char*) malloc(1);

	// Open HTTP connection
	if (!m_hInternet) 
	{
		std::cerr << "ERROR - IPCamera::Open:" << std::endl;
		std::cerr << "\t ... HTTP client not initialized.\n";
		return RET_FAILED;
	}

	std::cout << "INFO - IPCamera::Open:" << std::endl;
	std::cout << "\t ... Opening the following URL " << std::endl;
	std::cout << "\t ... " << m_url << std::endl;
	m_hURL = InternetOpenUrl(m_hInternet, m_url, NULL, 0, INTERNET_FLAG_NO_CACHE_WRITE, 0);
	if (!m_hURL) 
	{
		std::cerr << "ERROR - IPCamera::Open:" << std::endl;
		std::cerr << "\t ... Unable to setup URL" << std::endl;
		std::cerr << "\t ... " << GetLastError() << std::endl;
		return RET_FAILED;
	}

	// Get Content-Type
	char contentType[256];
	DWORD cbContentType = sizeof(contentType);
	if (HttpQueryInfo(m_hURL, HTTP_QUERY_CONTENT_TYPE,
			contentType, &cbContentType, NULL))
	{
		std::cout << "INFOR - IPCamera::Open:" << std::endl;
		std::cout << "\t ... ContentType is '" << contentType << "'." << std::endl;
		if (ExtractBoundaryString(contentType) & RET_FAILED)
		{
			std::cerr << "ERROR - IPCamera::Open:" << std::endl;
			std::cerr << "\t ... Unable to extract boundary string.\n";
			std::cerr << "\t ... from contentType '" << contentType << "'." << std::endl;
			return RET_FAILED;
		}
	}

	// Start image acquisition process
	if (m_mode == CONTINUOUS)
	{
		// Start the HTTP acquisition thread if it is not already running
		if (m_GetHTTPDataThread == 0)
		{
			m_startTime = GetTickCount(); // Set timestamp
			m_GetHTTPDataThread = new boost::thread(boost::bind(ThreadProcedure_GetHTTPData, this));

			//AfxBeginThread(ThreadProcedure_GetHTTPData, this);
		}
	}

	m_open = true;
	return RET_OK;
}

unsigned long IPCamera::ExtractBoundaryString(char *contentType)
{
	char *contentTypeCurrent, type[256], *typeCurrent, key[256], *keyCurrent;
	char value[256], *valueCurrent;

	// Extract boundary string from Content-Type
	contentTypeCurrent = contentType;
	typeCurrent = type;
	while ((*contentTypeCurrent != 0) && (*contentTypeCurrent != ';'))
	{
		*typeCurrent++ = *contentTypeCurrent;
		contentTypeCurrent++;
	}
	*typeCurrent = 0;
	if (*contentTypeCurrent == ';')
	{
		contentTypeCurrent++;
		while (*contentTypeCurrent == ' ')
			contentTypeCurrent++;
		keyCurrent = key;
		while ((*contentTypeCurrent != 0) && (*contentTypeCurrent != '='))
		{
			*keyCurrent++ = *contentTypeCurrent;
			contentTypeCurrent++;
		}
		*keyCurrent = 0;
		if (*contentTypeCurrent == '=')
		{
			contentTypeCurrent++;
			valueCurrent = value;
			while (*contentTypeCurrent != 0)
			{
				*valueCurrent++ = *contentTypeCurrent;
				contentTypeCurrent++;
			}
			*valueCurrent = 0;
			if (strcmp(key, "boundary") == 0)
			{
				strcpy(m_boundary, value);
				std::cout << "INFO - IPCamera::ExtractBoundaryString:" << std::endl;
				std::cout << "\t ... boundary string is '" << m_boundary << "'\n";
			}
			else
			{
				std::cerr << "ERROR - IPCamera::ExtractBoundaryString:" << std::endl;
				std::cerr << "\t ... key != 'boundary'.\n";
				return RET_FAILED;
			}
		}
		else
		{
			std::cerr << "ERROR - IPCamera::ExtractBoundaryString:" << std::endl;
			std::cerr << "\t ...  contentTypeCurrent != '='.\n";
			return RET_FAILED;
		}
	}
	else
	{
		std::cerr << "ERROR - IPCamera::ExtractBoundaryString:" << std::endl;
		std::cerr << "\t ...  contentTypeCurrent != ';'.\n";
		return RET_FAILED;
	}

	return RET_OK;
}

unsigned long IPCamera::Close()
{
	if (m_hURL)
	{
		InternetCloseHandle(m_hURL);
		m_hURL = 0;
	}

	if (m_hInternet != NULL)
	{
		// Close Internet connection
		InternetCloseHandle(m_hInternet);
		m_hInternet = 0;
	}

	if (m_pDynamicBuffer)
	{
		free(m_pDynamicBuffer);
		m_pDynamicBuffer = 0;
	}

	// HTTP acquisition thread is still running
	if (m_GetHTTPDataThread)
	{
		m_GetHTTPDataThread->interrupt();
		delete m_GetHTTPDataThread;
		m_GetHTTPDataThread = 0;
	}

	if (m_JpegDecoder)
	{
		delete m_JpegDecoder;
		m_JpegDecoder = 0;
	}

	if (m_url)
	{
		free(m_url);
	}

	if (m_pJPEGStreamQueue)
	{
		delete m_pJPEGStreamQueue;
		m_pJPEGStreamQueue = 0;
	}

	return RET_OK;
}

unsigned long IPCamera::GetColorImage(cv::Mat* colorImage)
{
	if (!m_open)
	{
		std::cerr << "ERROR - IPCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... Camera interface not open.\n";
		return RET_FAILED;
	}

	// Start the HTTP acquisition thread if it is not already running
	if (m_GetHTTPDataThread)
	{
		// implement me for single picture acquisition
	}
	else
	{
		std::cerr << "ERROR - IPCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... HTTP thread not initialized.\n";
		return RET_FAILED;
	}

	// Get an item from the queue
	char* pJPEG = 0; 
	int jpegSize = 0;
	if (m_pJPEGStreamQueue->Get(&pJPEG, jpegSize) & RET_OK)
	{
		//std::cout << "CameraAxisJPEGStreamQueue::Get: jpegSize is '" << jpegSize << "' .\n";
		//std::cout << "CameraAxisJPEGStreamQueue::Get: pJPEG is '" << pJPEG << "' .\n";
		// Lock critical section (weak lock, runtime conditions still possible)
		m_bProcessingCriticalSection = true;
		{
			m_JpegDecoder->setSource(pJPEG, jpegSize);
		    m_JpegDecoder->readHeader();

		    m_JpegDecoder->cvDecompress(colorImage);
		}
		m_bProcessingCriticalSection = false;

		free(pJPEG);
		return RET_OK;
	}
	else
	{
		return RET_FAILED;
	}
	return RET_OK;
}

unsigned long ipa_CameraSensors::ThreadProcedure_GetHTTPData(IPCamera* ipCamera)
{
	std::cout << "INFO - IPCamera::ThreadProcedure_GetHTTPData:" << std::endl;
	std::cout << "\t ... Reading HTTP data from camera.\n";
//************************************************************************
// Receive data
//************************************************************************
	if (!ipCamera->m_hURL)
	{
		std::cerr << "ERROR - IPCamera::ThreadProcedure_GetHTTPData:" << std::endl;
		std::cerr << "\t ... m_hURL is NULL.\n";
		return RET_FAILED;
	}

	char* buffer = (char*) malloc(ipCamera->m_bufferSize);
	for (;;)
	{
		// Exit loop if abort is requested
		boost::this_thread::interruption_point();

#ifndef __LINUX__
		// Process Windows messages
		MSG msg;
		while (PeekMessage(&msg, 0, 0, 0, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
#endif

		// Receive data
		DWORD bytesRead;
		bool received = false;

		long timeStart = GetTickCount();
		BOOL fResult = InternetReadFile(ipCamera->m_hURL,
			buffer, ipCamera->m_bufferSize, &bytesRead);
		long timeEnd = GetTickCount();
		long timeDiff = timeEnd-timeStart;
		if (fResult)
		{
			if (bytesRead > 0) // Note: Check if bytesRead == m_bufferSize
			{
				if (timeDiff > 0)
				{
					//std::cout << "IPCamera::GetColorImage:\n";
					//std::cout << "Received " << (int) bytesRead << " bytes in " << (int) timeDiff << " milliseconds => " << (int) (bytesRead*1000)/timeDiff << " bytes/sec\n";
				}

//************************************************************************
// Process data
//************************************************************************
				ThreadProcedureHelper_StoreData(buffer, ipCamera);

				received = true;
			}
		}
		if (!received)
		{
			break;
		}
	}
	free(buffer);

	InternetCloseHandle(ipCamera->m_hURL);
	return RET_FAILED;
}



unsigned long ipa_CameraSensors::ThreadProcedureHelper_StoreData(char *data, IPCamera* ipCamera)
{
	//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Received " << ipCamera->m_bufferSize << " bytes\n";
	// Copy new data into dynamic buffer that hold the partial downloaded jpeg image
	char* newBuffer = (char*) malloc(ipCamera->m_dynamicBufferLen + ipCamera->m_bufferSize);
	if (ipCamera->m_pDynamicBuffer) memcpy(newBuffer, ipCamera->m_pDynamicBuffer, ipCamera->m_dynamicBufferLen);
	memcpy(newBuffer + ipCamera->m_dynamicBufferLen, data, ipCamera->m_bufferSize);
	ipCamera->m_dynamicBufferLen += ipCamera->m_bufferSize;
	if (ipCamera->m_pDynamicBuffer) free(ipCamera->m_pDynamicBuffer);
	ipCamera->m_pDynamicBuffer = newBuffer;

	// Search boundary for beginning of JPEG image
	if (ipCamera->m_JPEGStart == -1)
	{
		//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Searching for beginning of JPEG image\n";

		char* pCurrent = ipCamera->m_pDynamicBuffer;
		while (pCurrent-ipCamera->m_pDynamicBuffer < ipCamera->m_dynamicBufferLen - (int) strlen(ipCamera->m_boundary))
		{
			if (ipCamera->m_pDynamicBuffer == NULL) return 0;

			if (*pCurrent == ipCamera->m_boundary[0])
			{
				if (memcmp(pCurrent, ipCamera->m_boundary, strlen(ipCamera->m_boundary)) == 0)
				{
					//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Recognized beginning of JPEG picture (1)\n";

					pCurrent += strlen(ipCamera->m_boundary);
					// Skipping HTTP header
					if (*pCurrent == 13)
					{
						pCurrent++;
						if (*pCurrent == 10)
						{
							pCurrent++;

							for (;;)
							{
								char header[256], *headerCurrent;
								headerCurrent = header;
								while (*pCurrent != 13)
								{
									*headerCurrent++ = *pCurrent;
									pCurrent++;
								}
								pCurrent++;
								if (*pCurrent == 10)
									pCurrent++;
								else
									break;
								*headerCurrent = 0;
								if (strlen(header) == 0)
									break;
							}

							ipCamera->m_JPEGStart = (int)(pCurrent-ipCamera->m_pDynamicBuffer);
							//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Recognized beginning of JPEG picture (2)\n";
							break;
						}
					}
				}
			}
			pCurrent++;
		}
	}
	if (ipCamera->m_JPEGStart == -1)
	{
		std::cerr << "ERROR - IPCamera::ThreadProcedureHelper_StoreData:" << std::endl;
		std::cerr << "\t ... Beginning of JPEG image not found\n";
		return 0;
	}

	// Search boundary of end of JPEG image
	char *pCurrent2 = ipCamera->m_pDynamicBuffer + ipCamera->m_JPEGStart + 1;
	while (pCurrent2 < ipCamera->m_pDynamicBuffer + ipCamera->m_dynamicBufferLen - (int) strlen(ipCamera->m_boundary))
	{
		if (*pCurrent2 == ipCamera->m_boundary[0])
		{
			if (memcmp(pCurrent2, ipCamera->m_boundary, strlen(ipCamera->m_boundary)) == 0)
			{
				//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Recognized end of JPEG picture\n";

				char *pJPEGEnd = pCurrent2;
				int jpegSize = (int)((pJPEGEnd-ipCamera->m_pDynamicBuffer)-ipCamera->m_JPEGStart-2);

				// JPEG image was received completely - process it

				long endTime = GetTickCount();
				long diffTime = endTime-ipCamera->m_startTime;
				//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Receiving JPEG image ( " << jpegSize << " bytes): " << diffTime << " ms\n";

				ipCamera->m_startTime = GetTickCount();

				// Check if it is worth providing the image
				if (!ipCamera->m_bProcessingCriticalSection)
					ipCamera->m_pJPEGStreamQueue->Add(ipCamera->m_pDynamicBuffer + ipCamera->m_JPEGStart, jpegSize);

				if (ipCamera->m_pDynamicBuffer == NULL) return 0;

				// Update buffer to reflect the remaining data
				int newBufferSize = (int)(ipCamera->m_dynamicBufferLen - (pCurrent2-ipCamera->m_pDynamicBuffer));
				char* newBuffer = (char*) malloc(newBufferSize);
				memcpy(newBuffer, pCurrent2, newBufferSize);
				free(ipCamera->m_pDynamicBuffer);
				ipCamera->m_pDynamicBuffer = newBuffer;
				ipCamera->m_dynamicBufferLen = newBufferSize;

				ipCamera->m_JPEGStart = -1;

				return 0;
			}
		}
		pCurrent2++;
	}
	return 0;
}

#endif // __LINUX__

