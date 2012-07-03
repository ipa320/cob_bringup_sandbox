/// @file TestCameraSensors.cpp
/// Definiert den Einstiegspunkt für die Konsolenanwendung.
/// @author Phillip Schmidt
/// @date 2009

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <Vision/CameraSensors/AbstractColorCamera.h>
#include <Vision/CameraSensors/AbstractRangeImagingSensor.h>
#include <Vision/CameraSensors/LibCameraSensorsTypes.h>

#include <math.h>

libCameraSensors::AbstractColorCamera* colorCamera = libCameraSensors::CreateColorCamera_AVTPikeCam();
libCameraSensors::AbstractRangeImagingSensor* rangeImagingSensor = libCameraSensors::CreateRangeImagingSensor_SR3000();

unsigned long Test()
{
	bool visualize = true;
	std::string RangeWindowName = "Range";
	std::string HoughWindowName = "Hough";

	IplImage* visualizationReferenceImage = cvLoadImage("Pictures/building.jpg");
	CvMemStorage* storage = cvCreateMemStorage(0);	/// Line endings storage
	CvSeq* lines = 0;
	int AngleBins = 45;	/// Controls angle resolution of Hough trafo (bins per pi)
	IplImage* Image = cvCreateImage(cvGetSize(visualizationReferenceImage), IPL_DEPTH_8U, 3);	/// Visualization image
	cvCopyImage(visualizationReferenceImage,Image);
	IplImage* GrayImage = cvCreateImage(cvGetSize(Image), IPL_DEPTH_8U, 1);
	cvCvtColor(Image, GrayImage, CV_RGB2GRAY);
	IplImage* CannyImage = cvCreateImage(cvGetSize(Image), IPL_DEPTH_8U, 1);	/// Edge image
	cvCanny(GrayImage, CannyImage, 25, 50);
	CvPoint ROIp1 = cvPoint(100,10);		/// Tablet ROI
	CvPoint ROIp2 = cvPoint(visualizationReferenceImage->width-40+ROIp1.x,visualizationReferenceImage->height-200+ROIp1.y);
	cvSetImageROI(CannyImage,cvRect(ROIp1.x,ROIp1.y,ROIp2.x-ROIp1.x,ROIp2.y-ROIp1.y));
	cvRectangle(Image, ROIp1, ROIp2, CV_RGB(0,255,0));
	//int maxd = cvRound(sqrt(sqrt((double)ROIp2.x-ROIp1.x)+sqrt((double)ROIp2.y-ROIp1.y)));	/// Maximum of possible distance value in Hough space
	int maxd = cvRound(sqrt((double)(((ROIp2.x-ROIp1.x)*(ROIp2.x-ROIp1.x))+((ROIp2.y-ROIp1.y)*(ROIp2.y-ROIp1.y)))));	/// Maximum of possible distance value in Hough space
	IplImage* HoughSpace = cvCreateImage(cvSize(maxd,AngleBins+1),IPL_DEPTH_8U, 1);		/// Hough space image (black=no line, white=there are lines at these bins)
	cvZero(HoughSpace);
	
	/// Hough transformation
	int AccumulatorThreshold = 100;		/// Threshold parameter. A line is returned by the function if the corresponding accumulator value is greater than threshold.
	double MinimumLineLength = 50;		/// For probabilistic Hough transform it is the minimum line length.
	double MaximumGap = 4;				/// For probabilistic Hough transform it is the maximum gap between line segments lieing on the same line to treat them as the single line segment (i.e. to join them).
	lines = cvHoughLines2(CannyImage, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/AngleBins, AccumulatorThreshold, MinimumLineLength, MaximumGap);
	
	for(int i = 0; i < lines->total; i++ )
	{
		CvPoint* line = (CvPoint*)cvGetSeqElem(lines,i);
		/// Endings of a line
		CvPoint p0 = cvPoint(line[0].x+ROIp1.x,line[0].y+ROIp1.y);
		CvPoint p1 = cvPoint(line[1].x+ROIp1.x,line[1].y+ROIp1.y);
		cvLine(Image, p0, p1, CV_RGB(255,0,0), 3, 8 );
		
		/// Slope/angle of line
		double phi = CV_PI/2;
		if(p0.x != p1.x) phi = atan((double)(p1.y-p0.y)/(double)(p1.x-p0.x));
		phi += (phi < 0)*CV_PI;

		/// Hessian normal form parameters: d = x*cos(alpha) + y*sin(alpha)
		/// with alpha in [0...pi], d in [0...maxd]
		double alpha = phi+CV_PI/2;
		alpha -= (alpha > CV_PI)*CV_PI;

		double d = p0.x;
		if(p0.x != p1.x)
		{
			double n = p1.y - (p1.y-p0.y)/(p1.x-p0.x) * p1.x;
			d = abs(n * cos(phi));
		}

		/// Write Line into Hough space
		cvLine(HoughSpace, cvPoint(cvRound(d),cvRound(alpha/CV_PI*AngleBins)),cvPoint(cvRound(d),cvRound(alpha/CV_PI*AngleBins)),CV_RGB(255,255,255));
	}
	if(visualize)
	{
		cvNamedWindow(RangeWindowName.c_str());
		cvNamedWindow(HoughWindowName.c_str());
		cvShowImage(RangeWindowName.c_str(), Image);
		cvShowImage(HoughWindowName.c_str(), HoughSpace);
		cvWaitKey(0);
	}
	cvCopyImage(Image,visualizationReferenceImage);

	cvReleaseImage(&GrayImage);
	cvReleaseImage(&CannyImage);

	/*
	IplImage* img1 = cvLoadImage("Cob3.jpg");
	IplImage* img2 = cvCreateImage(cvGetSize(img1),img1->depth,1);
	IplImage* img3 = cvCreateImage(cvGetSize(img1),img1->depth,1);
	IplImage* img4 = cvCreateImage(cvGetSize(img1),img1->depth,1);
	cvNamedWindow("Img1");
	cvNamedWindow("Img2");
	cvNamedWindow("Img3");
	cvCvtColor(img1, img2, CV_RGB2GRAY);
	cvCanny(img2, img3, 100, 200);
	cvShowImage("Img1", img1);
	cvShowImage("Img2", img2);
	cvShowImage("Img3", img3);
	cvWaitKey(0);
	cvReleaseImage(&img1);
	cvReleaseImage(&img2);
	cvReleaseImage(&img3);
	cvDestroyAllWindows();

	/*
	IplImage* Img1;
	IplImage* Img2;
	IplImage* Img3;

	cvNamedWindow("Img");
	while (cvGetWindowHandle("Img"))
	{
		if(cvWaitKey(10)=='q') break;

		/// Uncomment when using <code>GetColorImage</code> instead of <code>GetColorImage2</code>
	    //ColorImage = cvCreateImage(cvSize(1388,1038),IPL_DEPTH_8U,3);
		if (colorCamera->GetColorImage2(&Img1) == libCameraSensors::RET_FAILED)
		//if (colorCamera->GetColorImage(ColorImage, true) == libCameraSensors::RET_FAILED)
		{
			std::cerr << "TestCameraSensors: Color image acquisition failed\n";
			getchar();
			return ipa_utils::RET_FAILED;
		}

		if (colorCamera->GetColorImage2(&Img2) == libCameraSensors::RET_FAILED)
		{
			std::cerr << "TestCameraSensors: Color image acquisition failed\n";
			getchar();
			return ipa_utils::RET_FAILED;
		}

		Img3 = cvCreateImage(cvGetSize(Img1),Img1->depth,Img1->nChannels);
		cvSub(Img1, Img2, Img3);
		cvShowImage("Img", Img3);

		cvReleaseImage(&Img1);
		cvReleaseImage(&Img2);
		cvReleaseImage(&Img3);
	}*/

	return ipa_utils::RET_OK;
}

/// Get image from the color cam and show it.
/// @return Return code
unsigned long ShowColorImage()
{
	IplImage* ColorImage;

	cvNamedWindow("ColorCamera");
	while (cvGetWindowHandle("ColorCamera"))
	{
		if(cvWaitKey(10)=='q') break;

		/// Uncomment when using <code>GetColorImage</code> instead of <code>GetColorImage2</code>
	    //ColorImage = cvCreateImage(cvSize(1388,1038),IPL_DEPTH_8U,3);
		if (colorCamera->GetColorImage2(&ColorImage) == libCameraSensors::RET_FAILED)
		//if (colorCamera->GetColorImage(ColorImage, true) == libCameraSensors::RET_FAILED)
		{
			std::cerr << "TestCameraSensors: Color image acquisition failed\n";
			getchar();
			return ipa_utils::RET_FAILED;
		}

		cvShowImage("ColorCamera", ColorImage);

		cvReleaseImage(&ColorImage);
	}
	return ipa_utils::RET_OK;
}

/// Adjust gray scale.
/// @param Source original image
/// @param Dest destination for result image
void ConvertToShowImage(IplImage* Source, IplImage* Dest)
{
        double Min, Max;
        cvMinMaxLoc(Source, &Min, &Max);
		double w = Max-Min;
        for(int j=0; j<Source->height; j++)
		{
			for(int i=0; i<Source->width; i++)
			{
				double d = cvGetReal2D(Source, j, i);
				int V= (int)(255.0 * ((d-Min)/w));

				CvScalar Color = CV_RGB(V, V, V);
				cvSet2D(Dest, j, i, Color);
			}
		}
}

/// Get range image, convert and show it.
/// @return Return code
unsigned long ShowRangeImage()
{
	IplImage* RangeImage = cvCreateImage(cvSize(176, 144), IPL_DEPTH_32F, 1);

	cvNamedWindow("Range image");

	while(cvGetWindowHandle("Range image"))
	{
		if(cvWaitKey(10)=='q') break;

		/// Get image
		if(rangeImagingSensor->AcquireImages(RangeImage) & ipa_utils::RET_FAILED)
		{	
			std::cerr << "ShowRangeImage: Range image acquisition failed." << std::endl;
			return ipa_utils::RET_FAILED;	
		}

		/// Process image
		IplImage* OutputImage = cvCreateImage(cvSize(RangeImage->width, RangeImage->height), IPL_DEPTH_8U, 3);
		ConvertToShowImage(RangeImage, OutputImage);
		
		/// Show image
		cvShowImage("Range image", OutputImage);
	}
	cvReleaseImage(&RangeImage);

	return ipa_utils::RET_OK;
}

/// Show color and range image.
/// @return Return code
unsigned long ShowColorAndRangeImage() {

	IplImage* ColorImage;
	IplImage* RangeImage = cvCreateImage(cvSize(176, 144), IPL_DEPTH_32F, 1);

	cvNamedWindow("ColorCamera");
	cvNamedWindow("Range image");

	while (cvGetWindowHandle("ColorCamera") && cvGetWindowHandle("Range image"))
	{
		if(cvWaitKey(10)=='q') break;
		
	/// color image
		if (colorCamera->GetColorImage2(&ColorImage) & libCameraSensors::RET_FAILED)
		{
			std::cerr << "TestCameraSensors: Color image acquisition failed\n";
			return ipa_utils::RET_FAILED;
		}
		
		cvShowImage("ColorCamera",ColorImage);

		cvReleaseImage(&ColorImage);

	/// range image
		/// Get image
		if(rangeImagingSensor->AcquireImages(RangeImage) & ipa_utils::RET_FAILED)
		{	
			std::cerr << "ShowRangeImage: Range image acquisition failed." << std::endl;
			return ipa_utils::RET_FAILED;	
		}

		/// Process image
		IplImage* OutputImage = cvCreateImage(cvSize(RangeImage->width, RangeImage->height), IPL_DEPTH_8U, 3);
		ConvertToShowImage(RangeImage, OutputImage);
		
		/// Show image
		cvShowImage("Range image", OutputImage);

		cvReleaseImage(&OutputImage);
	}
	cvReleaseImage(&RangeImage);

	return ipa_utils::RET_OK;
}

/// Open and initialize range camera.
/// @return Return code
unsigned long InitAndOpenColorCamera()
{
	/// color camera init & open
	if (colorCamera->Init("ConfigurationFiles/") & libCameraSensors::RET_FAILED)
	{
		std::cerr << "main: Error while initializing color camera." << std::endl;
		return ipa_utils::RET_FAILED;	
	}

	if (colorCamera->Open() & libCameraSensors::RET_FAILED)
	{
		std::cerr << "main: Error while opening color camera." << std::endl;
		return ipa_utils::RET_FAILED;
	}


	return ipa_utils::RET_OK;
}

/// Open and initialize color camera.
/// @return Return code
unsigned long InitAndOpenRangeCamera()
{
	/// Range imaging sensor init & open	
	if (rangeImagingSensor->Init("ConfigurationFiles/") & libCameraSensors::RET_FAILED)
	{
		std::cerr << "main: Error while initializing range imaging sensor." << std::endl;
		return ipa_utils::RET_FAILED;	
	}

	if (rangeImagingSensor->Open() & libCameraSensors::RET_FAILED)
	{
		std::cerr << "main: Error while opening range imaging sensor." << std::endl;
		getchar();
		return ipa_utils::RET_FAILED;	
	}
	return ipa_utils::RET_OK;
}

/// Show color or range image or both.
/// @param argc/argv "c": only color image, "r": only range image, no arguments: both
/// @return Return code
unsigned long main(int argc, char* argv[])
{
	/// Open range and color camera when no parameters are passed
	if (argc == 1)
	{
		/// Color camera init & open
		if (InitAndOpenColorCamera() & libCameraSensors::RET_FAILED)
		{
			std::cerr << "main: Error while Initializing/Opening color camera." << std::endl;
			getchar();
			return ipa_utils::RET_FAILED;	
		}

		/// Range imaging sensor init & open	
		if (InitAndOpenRangeCamera() & libCameraSensors::RET_FAILED)
		{
			std::cerr << "main: Error while initializing/opening range imaging sensor." << std::endl;
			getchar();
			return ipa_utils::RET_FAILED;	
		}
			
		if (ShowColorAndRangeImage() & libCameraSensors::RET_FAILED)
		{
			std::cerr << "main: Error while showing images." << std::endl;
			getchar();
			return ipa_utils::RET_FAILED;
		}

		colorCamera->Close();
		rangeImagingSensor->Close();
	}

	// args
	else if (argc >= 2)
	{
		char* ptr;
		ptr = argv[1];

		// Color cam
		if (ptr[0] == 'c')
		{	
			/// Color camera init & open
 			if (InitAndOpenColorCamera() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error while Initializing/Opening color camera." << std::endl;
				getchar();
				return ipa_utils::RET_FAILED;;	
			}

			// shows camera property information
			if (colorCamera->PrintCameraInformation() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error" << std::endl;
				std::cerr << "\t ... Could not print camera (properties) information." << std::endl;
			}
			
			// shows color image (and handles window)
			if (ShowColorImage() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error while showing color image." << std::endl;
				getchar();
				return ipa_utils::RET_FAILED;
			}

			colorCamera->Close();
		}
		
		// range sensor
		else if (ptr[0] == 'r')
		{
			/// Range imaging sensor init & open	
			if (InitAndOpenRangeCamera() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error while initializing/opening range imaging sensor." << std::endl;
				getchar();
				return ipa_utils::RET_FAILED;	
			}

			if (ShowRangeImage() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error while showing range image." << std::endl;
				getchar();
				return ipa_utils::RET_FAILED;
			}

			rangeImagingSensor->Close();
		}

		// test
		if (ptr[0] == 't')
		{	
			/*// Color camera init & open
 			if (InitAndOpenColorCamera() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error while Initializing/Opening color camera." << std::endl;
				getchar();
				return ipa_utils::RET_FAILED;;	
			}*/

			if (Test() & libCameraSensors::RET_FAILED)
			{
				std::cerr << "main: Error while testing." << std::endl;
				getchar();
				return ipa_utils::RET_FAILED;
			}

			colorCamera->Close();
		}

		else
		{
			/// Output help message
			std::cout <<
				"usage: \t TestCameraSensors \n"
				"\t default \t Show color and range image \n"
				"\t c \t\t Show only color image \n"
				"\t r \t\t Show only range image \n"
			<< std::endl;
		}
	}

	return ipa_utils::RET_OK;
}


