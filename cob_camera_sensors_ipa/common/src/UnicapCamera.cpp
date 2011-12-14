#ifdef __LINUX__

#include <highgui.h>
#include "UnicapCamera.h"
#include <stdio.h>
#include <string.h>
#include <vector>
#include <math.h>
#include <assert.h>


UnicapCamera::UnicapCamera( )
{
	m_Handle = NULL;			
	m_Device = NULL;

	m_Format = NULL;
	
	m_CameraActive = false;
	m_Resolution = 0;
}

UnicapCamera::UnicapCamera(int res)
{
	m_Handle = NULL;			
	m_Device = NULL;

	m_Format = NULL;
	
	m_CameraActive = false;
	m_Resolution = res;
}

UnicapCamera::~UnicapCamera() 
{
	if (m_Handle)
		free(m_Handle);
	if (m_Device)
		free(m_Device);
	if (m_Format)
		free(m_Format);
}


int UnicapCamera::Open()
{
	unicap_device_t  device;
	unicap_handle_t  handle;
	unicap_format_t format;
	
	//Initialisation
	if (m_Device == NULL)
	{
		m_Device = (unicap_device_t *)malloc(sizeof(unicap_device_t));
		if (m_Device == NULL)
		{
			printf("UnicapCamera::Open: Error, no memory!\n");
			return ERROR_NO_MEMORY;
		}
	}
	
	if (m_Handle == NULL)
	{
		m_Handle = (unicap_handle_t *)malloc(sizeof(unicap_handle_t));
			if (m_Handle == NULL)
		{
			printf("UnicapCamera::Open: Error, no memory!\n");
			return ERROR_NO_MEMORY;
		}
	}
	if (m_Format == NULL)
	{
		m_Format = (unicap_format_t *)malloc(sizeof(unicap_format_t));
		if (m_Format == NULL)
		{
			printf("UnicapCamera::Open: Error, no memory!\n");
			return ERROR_NO_MEMORY;
		}
	}
	
	
	// Search camera devices
	
	if( !SUCCESS( unicap_enumerate_devices( NULL, &device, 0 ) ) )
	{
		printf("UnicapCamera::Open: No device found!\n");
		return ERROR_NO_CAMERAS_FOUND;
	}
	else
	{
		*m_Device = device;
		printf("UnicapCamera::Open: Device %s found, vendor: %s, controlled by %s, %s\n", m_Device->identifier, m_Device->vendor_name, m_Device->device, m_Device->cpi_layer);
	}

	/*
	  Acquire a handle to this device
	 */
	if( !SUCCESS( unicap_open( &handle, m_Device ) ) )
	{
		printf("UnicapCamera::Open: Failed to open device %s: %s\n", m_Device->identifier, strerror(errno));
		return ERROR_CAMERA_COULD_NOT_BE_OPENED;
	}
	else
	{
		*m_Handle = handle;
		m_CameraActive = true;
		printf("DUnicapCamera::Open:evice %s successfully opened!\n", m_Device->identifier);
		
	}
	
	
	//Set format according to specified resolution	
	if( !SUCCESS( unicap_enumerate_formats( *m_Handle, NULL, &format, m_Resolution ) ) )
	{
		printf("UnicapCamera::Open: Failed to get video format, setting to default\n" );
			
		
		//return UNSPECIFIED_ERROR;;
	}
	else
	{
		*m_Format = format;
		printf("UnicapCamera::Open: Format %s chosen\n", format.identifier );
		printf("UnicapCamera::Open: Setting video format: \nwidth: %d\nheight: %d\nbpp: %d\nFOURCC: %c%c%c%c\n\n", \
		m_Format->size.width,\
		m_Format->size.height,\
		m_Format->bpp, \
		m_Format->fourcc & 0xff, \
		( m_Format->fourcc >> 8 ) & 0xff, \
		( m_Format->fourcc >> 16 ) & 0xff, \
		( m_Format->fourcc >> 24 ) & 0xff  \
		);
		/*
		Set this video format
		*/
		if( !SUCCESS( unicap_set_format( *m_Handle, m_Format ) ) )
		{
			printf("UnicapCamera::Open: Failed to set video format\n" );
			return UNSPECIFIED_ERROR;
		}
	}
	
	unicap_property_t property;

	// Set white balance mode to auto
        strcpy( property.identifier, "white_balance_mode" );
        if( !SUCCESS( unicap_get_property( *m_Handle, &property ) ) )
        {
                fprintf( stderr, "UnicapCamera::Open: Failed to get WB mode property\n" );
                exit( 1 );
        }

        // Set auto on this property
        property.flags = UNICAP_FLAGS_AUTO;//UNICAP_FLAGS_MANUAL;
        unicap_set_property( handle, &property );
	
	if( !SUCCESS( unicap_set_property( *m_Handle, &property ) ) )
        {
                printf( "UnicapCamera::Open: Failed to set property!\n" );
		return UNSPECIFIED_ERROR;
        }
	
	if( !SUCCESS( unicap_get_property( *m_Handle, &property ) ) )
        {
                fprintf( stderr, "UnicapCamera::Open: Failed to get WB mode property\n" );
                exit( 1 );
        }

        printf( "UnicapCamera::Open: Current white balance mode: %s\n", property.flags & UNICAP_FLAGS_AUTO ? "AUTO" : "MANUAL" );
	
	
	
	if( !SUCCESS( unicap_start_capture( *m_Handle ) ) )
		{
			printf( "UnicapCamera::Open: Failed to start capture on device %s\n", m_Device->identifier );
			return   UNSPECIFIED_ERROR;
		}
		
		

		
	return OK;
	
}

int UnicapCamera::Close(void)
{
	if( !SUCCESS( unicap_stop_capture( *m_Handle ) ) )
	{
		printf( "Failed to stop capture on device: %s\n", m_Device->identifier );
	}
	
	if( !SUCCESS( unicap_close( *m_Handle ) ) )
	{
		printf("Failed to close the device: %s\n", m_Device->identifier );
		return UNSPECIFIED_ERROR;
	}
	else
	{
		printf("UnicapCamera: %s closed\n",m_Device->identifier);
		return 0;
	}
}

int UnicapCamera::SetColorMode(int colorMode)
{
				m_ColorMode = colorMode;
				return 0;
}

int UnicapCamera::ShowAvailableVideoFormats()
{

	unicap_format_t format;
	for (int n=0;n < 10;n++)
	{

		if( !SUCCESS( unicap_enumerate_formats( *m_Handle, NULL, &format, n ) ) )
		{
			printf("Failed to get video format\n" );
			return UNSPECIFIED_ERROR;;
		}
		else
		{
			printf("Format %i: fcc: %i  bpp:%i \n",n,format.fourcc,format.bpp);
			printf(" h_stepping: %i v_stepping %i \n",format.h_stepping,format.v_stepping);
			printf("size_count %i \n",format.size_count);
		}
	}
	return 1;
}

int UnicapCamera::ConvertImage(cv::Mat* inputImg, unicap_data_buffer_t * inputRawBufferData)
{
	if (m_ColorMode == 0)
	{
		//printf("Kamera im RGB-Modus, keine Konvertierung notwendig; Puffer-Daten werden kopiert");
		ConvRGBIplImage(inputImg, inputRawBufferData);
	}
	
	if (m_ColorMode == 1)
	{
		//printf("Kamera im Ymono-Modus, eine Konvertierung wird vorgenommen");	
	}

	if (m_ColorMode == 2)
	{
		//printf("Kamera im YVV411-Modus, eine Konvertierung wird vorgenommen");	
	}

	if (m_ColorMode == 3)
	{
		//printf("Kamera im YVV422-Modus, eine Konvertierung wird vorgenommen");
		ConvUYVY2IplImage(inputImg, inputRawBufferData);		
	}

	return m_ColorMode;

}
int UnicapCamera::ConvRGBIplImage(cv::Mat* Img, unicap_data_buffer_t * rawBufferData)
{
	if (!m_Format)
	{
		printf("UnicapCamera::ConvUYVY2IplImage: Error, no format set\n");
		return ERROR_NO_FORMAT_SET;
	}
	
	float r=0, g=0, b=0;
	
	unsigned int bufIndex = 0;
	
	unsigned char * pBuff=(unsigned char*)img->data;
	long Pos=0;	
		
	while ( bufIndex < rawBufferData->buffer_size)
	{ 
		r = (float)(rawBufferData->data[bufIndex++]);
		g = (float)(rawBufferData->data[bufIndex++]);
		b = (float)(rawBufferData->data[bufIndex++]);
		
		pBuff[Pos++]=(unsigned char)b;
		pBuff[Pos++]=(unsigned char)g;
		pBuff[Pos++]=(unsigned char)r;	

	}
	return bufIndex; // return the bufferindex; this value isn't used, it is only for the return statement

}

int UnicapCamera::ConvUYVY2IplImage(cv::Mat* Img, unicap_data_buffer_t * rawBufferData)
{
	if (!m_Format)
	{
		printf("UnicapCamera::ConvUYVY2IplImage: Error, no format set\n");
		return ERROR_NO_FORMAT_SET;
	}
	
	float u,y,v;
	float r,g,b;
	
	unsigned int bufIndex = 0;

	unsigned char * pBuff=(unsigned char*)img->data;
	long Pos=0;	
		
	while ( bufIndex < rawBufferData->buffer_size)
	{ 
		u = (float)(rawBufferData->data[bufIndex++]);
		y = (float)(rawBufferData->data[bufIndex++]);
		v = (float)(rawBufferData->data[bufIndex++]);
		
		
		//Convert uyv to rgb
		y-=16;
		v-=128;u-=128;
			
		r = 1.164*y + 1.596*v;
		g = 1.164*y - 0.813*v - 0.391*u;
		b = 1.164*y   + 2.018*u ;

		if (r > 255) r=255;if (r < 0) r=0;
		if (g > 255) g = 255;if (g < 0)g=0;
		if (b > 255) b=255;if (b <0)b=0;
		pBuff[Pos++]=(unsigned char)b;
		pBuff[Pos++]=(unsigned char)g;
		pBuff[Pos++]=(unsigned char)r;	

		//  UYVY Format
		y = (float)(rawBufferData->data[bufIndex++] & 0x00FF);
		y-=16;

		r = 1.164*y  + 1.596*v ;
		g = 1.164*y  - 0.813*v - 0.391*u;
		b = 1.164*y   + 2.018*u ;

		if (r > 255) r=255;if (r < 0) r=0;
		if (g > 255) g = 255;if (g < 0) g=0;
		if (b > 255) b=255;if (b < 0) b=0;
		pBuff[Pos++]=(unsigned char)b;
		pBuff[Pos++]=(unsigned char)g;
		pBuff[Pos++]=(unsigned char)r;
	}
	return bufIndex; // return the bufferindex; this value isn't used, it is only for the return statement
}

int UnicapCamera::SetProperty(int propertyNo, int value )
{
	unicap_property_t property;
	unicap_property_t property_spec;
	
	
	unicap_void_property( &property_spec );

	if (!IsCameraActive())
	{
		printf("UnicapCamera::SetProperties: Please call open first! \n");
		return ERROR_NOT_OPENED;
	
	}
	
		
 	if( !SUCCESS( unicap_enumerate_properties( *m_Handle, &property_spec, &property, propertyNo ) ) )
        {
                printf( "UnicapCamera::SetProperties: Failed to enumerate property\n" );
                return UNSPECIFIED_ERROR;
        }
        
	
        property.value = value;
		printf("UnicapCamera::SetProperties: Setting property %d: %s to %f ... \n",propertyNo, property.identifier,property.value);
        if( !SUCCESS( unicap_set_property( *m_Handle, &property ) ) )
        {
                printf( "UnicapCamera::SetProperties: Failed to set property!\n" );
		return UNSPECIFIED_ERROR;
        }
	return OK;


}

int UnicapCamera::PrintCameraInformation()
{
   unicap_format_t format;
	 unicap_property_t property;
   int property_count;
   int format_count;

	if (!IsCameraActive())
	{
		printf("UnicapCamera::SetProperties: Please call open first! \n");
		return ERROR_NOT_OPENED;
	
	}

   unicap_reenumerate_properties( *m_Handle, &property_count );
   unicap_reenumerate_formats( *m_Handle, &format_count );
   
   printf( "Device: %s\n", m_Device->identifier );

   printf( "\tProperties[%d]:\n", property_count );
   
   for(int j = 0; SUCCESS( unicap_enumerate_properties( *m_Handle, NULL, &property, j ) ); j++ )
   {
       printf( "\t\t%s\n", property.identifier );
   }
   
   printf( "\tFormats[%d]:\n", format_count );

   for(int j = 0; SUCCESS( unicap_enumerate_formats( *m_Handle, NULL, &format, j ) ); j++ )
   {
       printf( "\t\t%s\n", format.identifier );
   }

   return OK;
}

int UnicapCamera::GetColorImage(cv::Mat* img, char* FileName )
{
	unsigned char *image_buffer = NULL;
	unicap_data_buffer_t buffer;
	unicap_data_buffer_t *returned_buffer;
	int error = 0;

	// Initialize IPL image
	CV_Assert(img != 0);
	img->create(m_Format->size.height, m_Format->size.width, CV_8UC3);

	
	// Initialize the image buffer
	memset( &buffer, 0x0, sizeof( unicap_data_buffer_t ) );
	
	if (!m_Format)
	{
		printf("UnicapCamera::Acquire: No format set!\n");
		error =  ERROR_NO_FORMAT_SET;
	}

	if ((error == 0) && (!m_Handle))
	{
		printf("UnicapCamera::Acquire: No Camera handle available.\n");
		error =   ERROR_NOT_OPENED;
	}
	
	// Allocate memory for the image buffer
	if (error == 0)
	{
		if( !( image_buffer = (unsigned char *)malloc( m_Format->buffer_size ) ) )
		{
			printf("UnicapCamera::Acquire: Failed to allocate %d bytes\n" );
			error =   ERROR_NO_MEMORY;
		}

		buffer.data = image_buffer;
		buffer.buffer_size = m_Format->buffer_size;
	}
	
	// Start the capture process on the device
	// Queue the buffer
	// The buffer now gets filled with image data by the capture device
	if (error == 0)
	{
		if( !SUCCESS( unicap_queue_buffer( *m_Handle, &buffer ) ) )
		{
			printf("UnicapCamera::Acquire: Failed to queue a buffer on device: %s\n", m_Device->identifier );
			error =   UNSPECIFIED_ERROR;
		}
	}
	
	// Wait until the image buffer is ready
	if (error == 0)
	{
		if( !SUCCESS( unicap_wait_buffer( *m_Handle, &returned_buffer ) ) )
		{
			printf("UnicapCamera::Acquire: Failed to wait for buffer on device: %s\n", m_Device->identifier );
			error =   UNSPECIFIED_ERROR;
		}
	}

	// Stop the device
	if (error == 0)
	{
		if( !returned_buffer->buffer_size )
		{
			printf("UnicapCamera::Acquire: Returned a buffer size of 0!\n" );
			error = UNSPECIFIED_ERROR;
		}
	}
	if (error == 0)
	{
		ConvertImage(img, returned_buffer);
	}

	if ((error==0) && (FileName)) cvSaveImage(FileName, Img);
	if (image_buffer)
	{
		free( image_buffer );
	}
		
	return error;
}

#endif // __LINUX__
