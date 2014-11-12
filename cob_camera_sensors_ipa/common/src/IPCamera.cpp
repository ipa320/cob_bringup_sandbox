#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/IPCamera.h"

	#include <stdio.h>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/IPCamera.h"
#endif


using namespace ipa_CameraSensors;

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
	m_dynamicBufferLen = 0;
	m_curlHandle = NULL;
	m_pJPEGStreamQueue = 0;
	m_state = ST_START;
	printf("constr m_state=%d this=%p\n", m_state, this);
	m_writeBufInd = 0;
	m_readBufInd = -1;
	m_newestBufInd = -1;
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

	curl_global_init(CURL_GLOBAL_ALL);
	m_curlHandle = curl_easy_init();
	m_boundaryExtracted = false;

	m_initialized = true;
	m_state = ST_START;
	printf("init m_state=%d this=%p\n", m_state, this);
	m_readBufInd = -1;
	m_newestBufInd = -1;
	m_jpegLength = 0;
	return RET_OK;
}

unsigned long IPCamera::Open()
{
	int ret;
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
	if (!m_curlHandle) 
	{
		std::cerr << "ERROR - IPCamera::Open:" << std::endl;
		std::cerr << "\t ... HTTP client not initialized.\n";
		return RET_FAILED;
	}

	std::cout << "INFO - IPCamera::Open:" << std::endl;
	std::cout << "\t ... Opening the following URL " << std::endl;
	std::cout << "\t ... " << m_url << std::endl;

	curl_easy_setopt(m_curlHandle, CURLOPT_NOPROGRESS, 1L);
	curl_easy_setopt(m_curlHandle, CURLOPT_WRITEFUNCTION, WriteFunction);
	curl_easy_setopt(m_curlHandle, CURLOPT_WRITEDATA, this);
	ret = curl_easy_setopt(m_curlHandle, CURLOPT_URL, m_url);
	if (ret)
	{
		std::cerr << "ERROR - IPCamera::Open:" << std::endl;
		std::cerr << "\t ... Unable to setup URL" << std::endl;
		std::cerr << "\t ... Error " << ret << std::endl;
		return RET_FAILED;
	}

	// Start image acquisition process
	if (m_mode == CONTINUOUS)
	{
		// Start the HTTP acquisition thread if it is not already running
		if (m_GetHTTPDataThread == 0)
		{
			m_GetHTTPDataThread = new boost::thread(boost::bind(ThreadProcedure_GetHTTPData, this));
		}
	}

	m_readBufInd = -1;
	m_newestBufInd = -1;
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
	curl_easy_cleanup(m_curlHandle);

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
	//printf("#####################################################\n");
	if (!m_open)
	{
		std::cerr << "ERROR - IPCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... Camera interface not open.\n";
		return RET_FAILED;
	}

	if (m_newestBufInd == -1)
	{
		std::cerr << "ERROR - IPCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... No image available yet.\n";
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

	{
		boost::mutex::scoped_lock lock(m_bufferMutex);
		m_readBufInd = m_newestBufInd;
	}

	m_JpegDecoder->setSource(reinterpret_cast<char *>(&m_jpegBuffer[m_readBufInd][0]),
		m_jpegBuffer[m_readBufInd].size());
	m_JpegDecoder->readHeader();
	m_JpegDecoder->cvDecompress(colorImage);

	/*
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
	*/


	return RET_OK;
}

unsigned long ipa_CameraSensors::ThreadProcedure_GetHTTPData(IPCamera* ipCamera)
{
	std::cout << "INFO - IPCamera::ThreadProcedure_GetHTTPData:" << std::endl;
	std::cout << "\t ... Reading HTTP data from camera.\n";
	//************************************************************************
	// Receive data
	//************************************************************************
	curl_easy_perform(ipCamera->m_curlHandle);
	return RET_FAILED;
}



unsigned long ipa_CameraSensors::ThreadProcedureHelper_StoreData(char *data, IPCamera* ipCamera)
{
	//std::cout << "IPCamera::ThreadProcedureHelper_ProcessData: Received " << ipCamera->m_bufferSize << " bytes\n";
	// Copy new data into dynamic buffer that hold the partial downloaded jpeg image

	char* newBuffer = (char*) malloc(ipCamera->m_dynamicBufferLen + ipCamera->m_bufferSize);
	if (ipCamera->m_pDynamicBuffer)
		memcpy(newBuffer, ipCamera->m_pDynamicBuffer, ipCamera->m_dynamicBufferLen);
	memcpy(newBuffer + ipCamera->m_dynamicBufferLen, data, ipCamera->m_bufferSize);
	ipCamera->m_dynamicBufferLen += ipCamera->m_bufferSize;
	if (ipCamera->m_pDynamicBuffer)
		free(ipCamera->m_pDynamicBuffer);
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

size_t IPCamera::WriteFunction(void *ptr, size_t size, size_t nmemb, void *pIPCamera)
{
	return static_cast<IPCamera *>(pIPCamera)->WriteMethod(reinterpret_cast<unsigned char *>(ptr), size*nmemb);
}


size_t IPCamera::WriteMethod(unsigned char *buffer, size_t size)
{
	char line[100], *contentType, ch;
	size_t bufferIndex;

	//printf("WriteMethod: size=%d\n", size);

	for (bufferIndex=0; bufferIndex<size; )
	{
		//printf("write m_state=%d this=%p\n", m_state, this);
		switch(m_state)
		{
		case ST_START:
			curl_easy_getinfo(m_curlHandle, CURLINFO_CONTENT_TYPE, &contentType);
			if (ExtractBoundaryString(contentType) & RET_FAILED)
			{
				std::cerr << "ERROR - IPCamera::Open:" << std::endl;
				std::cerr << "\t ... Unable to extract boundary string.\n";
				std::cerr << "\t ... from contentType '" << contentType << "'." << std::endl;
				return 0;
			}
			m_index = 0;
			m_badBoundary = 0;
			//printf("m_boundary='%s'\n", m_boundary);
			m_state = ST_SEARCH_BOUNDARY;
			break;

		case ST_SEARCH_BOUNDARY:
			if (buffer[bufferIndex++] == m_boundary[m_index])
			{
				if (m_boundary[++m_index] == '\0')
				{
					line[0] = 'x';
					m_index = 1;
					if (m_badBoundary > 4)
						printf("##################### %d bad boundary chars %d\n", m_badBoundary, m_jpegLength);
					m_badBoundary = 0;
					m_state = ST_READ_LINE;
				}
			}
			else
			{
				m_badBoundary++;
				m_index = 0;
			}
			break;

		case ST_READ_LINE:
			ch = buffer[bufferIndex++];
			if (m_index < sizeof line)
				line[m_index++] = ch;
			if (ch == '\n')
			{
				if(m_index > 2)
				{
					//printf("%.*s", m_index, line);
					sscanf(line, "Content-Length: %d", &m_jpegLength);
					//printf("m_jpeglength=%d\n", m_jpegLength);
					m_index = 0;
				}
				else
				{
					m_jpegBuffer[m_writeBufInd].clear();
					m_state = ST_COPY_DATA;
				}
			}
			break;

		case ST_COPY_DATA:
			m_jpegBuffer[m_writeBufInd].push_back(buffer[bufferIndex++]);
			if (m_jpegBuffer[m_writeBufInd].size() == m_jpegLength)
			{
				{
					boost::mutex::scoped_lock lock(m_bufferMutex);
					m_newestBufInd = m_writeBufInd;
					m_writeBufInd = m_readBufInd==-1 ? 2-m_writeBufInd : 3-m_writeBufInd-m_readBufInd;
				}
				//printf("m_writeBufInd = %d\n", m_writeBufInd);
				m_state = ST_SEARCH_BOUNDARY;
			}
			break;
		}
	}
	return size;
}
