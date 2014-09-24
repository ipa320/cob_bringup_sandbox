/// @file IPCamera.h
/// Interface to an IP camera like Axis 2100.
/// @author Jan Fischer
/// @date October, 2008.

#ifndef __IPA_IPCAMERA_H__
#define __IPA_IPCAMERA_H__

#ifdef __LINUX__
	#include "cob_vision_utils/CameraSensorDefines.h"
	#include "cob_vision_utils/CameraSensorTypes.h"
	#include "cob_vision_utils/GlobalDefines.h"
	#include "cob_vision_utils/memJpegDecoder.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorDefines.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/CameraSensorTypes.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
	#include "cob_object_perception_intern/windows/src/extern/MemJpegDecoder/memJpegDecoder.h"
#endif

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/bind.hpp>

#include <curl/curl.h>
#include <curl/types.h>
#include <curl/easy.h>

namespace ipa_CameraSensors {
typedef struct _CameraAxisJPEGStreamQueueEntry
{
	char *data;
	int dataLen;
	struct _CameraAxisJPEGStreamQueueEntry *next;
} CameraAxisJPEGStreamQueueEntry;


class __DLL_LIBCAMERASENSORS__ CameraAxisJPEGStreamQueue
{
	public:
		CameraAxisJPEGStreamQueue();
		~CameraAxisJPEGStreamQueue();

		unsigned long Add(char *data, int dataLen);
		unsigned long Get(char **data, int &dataLen);

	private:
		CameraAxisJPEGStreamQueueEntry *m_first, *m_last;
		boost::mutex m_StreamQueueMutex;
		boost::condition_variable m_ConditionPictureAvailable;
};

/// @ingroup ColorCameraDriver
/// Class retrieves an mJPEG stream from a camera and extractes the single
/// JPEG pictures from the stream. Both tasks are scheduled in separate threads.
class __DLL_LIBCAMERASENSORS__ IPCamera
{
	enum t_mode
	{
		CONTINUOUS, ///< Grab continuously images.
		SINGLE		///< Grab only a single image.
	};

	public:
		/// Constructor.
		IPCamera();
		~IPCamera();

		unsigned long Init(int bufferSize, const char* url, t_mode mode = CONTINUOUS);
		unsigned long Open();
		unsigned long Close();
		unsigned long GetColorImage(cv::Mat* img);

		/// Extracts the boundary string for an HTTP Stream to determine image boundaries.
		/// @param contentType The content type of the HTTP stream.
		/// @return Return code.
		unsigned long ExtractBoundaryString(char *contentType);

	public:
    CURL *m_curlHandle;
		t_mode m_mode;				///< CONTINUOUS or SINGLE picture acquisition mode

		memJpegDecoder* m_JpegDecoder;		///< Decodes JPEG image to bitmap image
		int m_bufferSize;			///< Buffer size of HTTP connection
		char* m_url;				///< URL to Axis 2100 camera
		char m_boundary[256];		///< Boundary string within HTTP stream
		char* m_pDynamicBuffer;		///< Buffer to store data from the HTTP stream until a full JPEG image is received
		int m_dynamicBufferLen;		///< Length of m_pDynamicBuffer
		int m_JPEGStart;			///< Specifies if the beginning of a JPEG image has been detected within the data stream
		CameraAxisJPEGStreamQueue* m_pJPEGStreamQueue; ///< Stores data of whole JPEG pictures within a queue.
		bool m_bProcessingCriticalSection;			///< Variable controls concurrent access to the image queue.
		bool m_bImageAcquisitionIsRunning; ///< Specifies if image acquisition thread is running;

		long m_startTime;			///< Timestamp for time measurements

		boost::thread* m_GetHTTPDataThread;

		bool m_initialized;
		bool m_open;
		
  private:
    static size_t WriteFunction(void *ptr, size_t size, size_t nmemb, void *pIPCamera);
    size_t WriteMethod(unsigned char *buffer, size_t size);
    bool m_boundaryExtracted;
    enum { ST_START, ST_SEARCH_BOUNDARY, ST_READ_LINE, ST_COPY_DATA } m_state;
    std::vector<unsigned char> m_jpegBuffer[3];
    int m_writeBufInd, m_readBufInd, m_newestBufInd;
    size_t m_jpegLength, m_index;
    int m_badBoundary;
		boost::mutex m_bufferMutex;
};

/// Retrieves HTTP data from the camera.
/// Procedure is executed within a thread to enable continuous retrieval of image information.
/// HTTP data is retrieved in portions of <code>m_bufferSize</code> bytes. Each piece
/// of received HTTP data is forwarded to <code>ThreadProcedureHelper_StoreData</code>.
/// @param pParam <code>this*</code> pointer.
unsigned long ThreadProcedure_GetHTTPData(IPCamera* ipCamera);

/// Merges single image data pieces to a whole image.
/// @param data Pointer to the received data from the HTTP stream.
/// @param dataLen Length of received data.
unsigned long ThreadProcedureHelper_StoreData(char *data, IPCamera* ipCamera);

} // namespace ipa_CameraSensors

#endif // __IPA_IPCAMERA_H__
