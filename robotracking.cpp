#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>

//new
#include <cv.h>
#include <highgui.h>
#include "time.h"

extern "C" {
    #include "bcm_host.h"
    #include "interface/vcos/vcos.h"

    #include "interface/mmal/mmal.h"
    #include "interface/mmal/mmal_logging.h"
    #include "interface/mmal/mmal_buffer.h"
    #include "interface/mmal/util/mmal_util.h"
    #include "interface/mmal/util/mmal_util_params.h"
    #include "interface/mmal/util/mmal_default_components.h"
    #include "interface/mmal/util/mmal_connection.h"

    #include "RaspiCamControl.h"
    #include "RaspiPreview.h"
    #include "RaspiCLI.h"
}

#include <semaphore.h>


// OPENCV
#include <iostream>
#include <fstream>
#include <sstream>
#include "time.h"

#include "opencv2/core/core.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "/home/xuyenngo/libfacerec/include/facerec.hpp"    //<-- to modify

using namespace cv;
using namespace std;


class Symbol {

public:
	Mat img;
	string name;

};

void sortCorners(std::vector<cv::Point2f>& corners, cv::Point2f center) {
	std::vector<cv::Point2f> top, bot;

	for (unsigned int i = 0; i < corners.size(); i++) {
		if (corners[i].y < center.y)
			top.push_back(corners[i]);
		else
			bot.push_back(corners[i]);
	}

	cv::Point2f tl = top[0].x > top[1].x ? top[1] : top[0];
	cv::Point2f tr = top[0].x > top[1].x ? top[0] : top[1];
	cv::Point2f bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
	cv::Point2f br = bot[0].x > bot[1].x ? bot[0] : bot[1];

	corners.clear();
	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
}

int readRefImages(Symbol *symbols) {

	symbols[0].img = imread("arrowL.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[0].img.data)
		return -1;
	threshold(symbols[0].img, symbols[0].img, 100, 255, 0);
	symbols[0].name = "Left 90";

	symbols[1].img = imread("arrowR.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[1].img.data)
		return -1;
	threshold(symbols[1].img, symbols[1].img, 100, 255, 0);
	symbols[1].name = "Right 90";

	symbols[2].img = imread("arrowT.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[2].img.data)
		return -1;
	threshold(symbols[2].img, symbols[2].img, 100, 255, 0);
	symbols[2].name = "Turn Around";

	symbols[3].img = imread("arrowB.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[3].img.data)
		return -1;
	threshold(symbols[3].img, symbols[3].img, 100, 255, 0);
	symbols[3].name = "Ball";

	symbols[4].img = imread("arrowL45.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[4].img.data)
		return -1;
	threshold(symbols[4].img, symbols[4].img, 100, 255, 0);
	symbols[4].name = "Left 45";

	symbols[5].img = imread("arrowR45.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[5].img.data)
		return -1;
	threshold(symbols[5].img, symbols[5].img, 100, 255, 0);
	symbols[5].name = "Right 45";

	symbols[6].img = imread("arrowStop.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[6].img.data)
		return -1;
	threshold(symbols[6].img, symbols[6].img, 100, 255, 0);
	symbols[6].name = "Stop";

	symbols[7].img = imread("arrowStop.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	if (!symbols[7].img.data)
		return -1;
	threshold(symbols[7].img, symbols[7].img, 100, 255, 0);
	symbols[7].name = "Go";

	return 0;

}
int lowThreshold;

void CannyThreshold(int, void*) {
}

// for debug and trace
#define TRACE 1
#define DEBUG_MODE 1
#define DEBUG if (DEBUG_MODE==1)

string fn_haar;
string fn_csv;
int im_width;    // image width
int im_height;    // image height
int PREDICTION_SEUIL ;
char key;

bool shouldExit = false;
uint totalVisitors   = 0;
uint currentVisitors = 0;

Mat gray,frame;
///////////////////////

Symbol symbols[10];

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 10
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE = 30000000; // 30Mbits/s


// variable to convert I420 frame to IplImage
int nCount=0;
IplImage *py, *pu, *pv, *pu_big, *pv_big, *image,* dstImage;


int mmal_status_to_int(MMAL_STATUS_T status);


string intToString(int number) {

	std::stringstream ss;
	ss << number;
	return ss.str();
}


/** Structure containing all state information for the current run
 */
typedef struct
{
   int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
   int width;                          /// Requested width of image
   int height;                         /// requested height of image
   int bitrate;                        /// Requested bitrate
   int framerate;                      /// Requested frame rate (fps)
   int graymode;    /// capture in gray only (2x faster)
   int immutableInput;      /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
                                       /// the camera output or the encoder output (with compression artifacts)
   RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
   RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

   MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
   MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
   MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
   MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

   MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port

} RASPIVID_STATE;

/** Struct used to pass information in encoder port userdata to callback
 */
typedef struct
{
   FILE *file_handle;                   /// File handle to write buffer data to.
   VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
   RASPIVID_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

////////////////////////////////////////
/////////////////////////////////////////////////
// trace if TRACE==1
/////////////////////////////////////////////////
void trace(string s)
{
    if (TRACE==1)
    {
        clog<<s<<"\n";
    }
}

/////////////////////////////////////////

// default status
static void default_status(RASPIVID_STATE *state)
{
   if (!state)
   {
      vcos_assert(0);
      return;
   }

   // Default everything to zero
   memset(state, 0, sizeof(RASPIVID_STATE));

   // Now set anything non-zero
   state->timeout     = 65000;     // capture time : here 65 s
   //state->width     = 320;      // use a multiple of 320 (640, 1280)
   //state->height     = 240;    // use a multiple of 240 (480, 960)
   state->width     = 320;      // use a multiple of 320 (640, 1280)
   state->height     = 240;    // use a multiple of 240 (480, 960)
   state->bitrate     = 17000000; // This is a decent default bitrate for 1080p
   state->framerate     = VIDEO_FRAME_RATE_NUM;
   state->immutableInput     = 1;
   state->graymode     = 0;    //gray by default, much faster than color (0), mandatory for face reco

   // Setup preview window defaults
   raspipreview_set_defaults(&state->preview_parameters);

   // Set up the camera_parameters to default
   raspicamcontrol_set_defaults(&state->camera_parameters);
}

void threshImage( Mat input, Mat output, int serial ) {
	Mat hsvImg = Mat::zeros( input.size(), input.type() );
	vector<Vec3f> circles;

	blur( input, hsvImg, Size( 9, 9 ) );
	cvtColor( hsvImg, hsvImg, CV_BGR2HSV );
	//inRange( hsvImg, Scalar( 70, 30, 30 ), Scalar( 100, 255, 255 ), output );
	inRange( hsvImg, Scalar( 70, 30, 30 ), Scalar( 100, 255, 255 ), output );

	HoughCircles( output, circles, CV_HOUGH_GRADIENT, 2, output.rows/4, 100, 40, 10, 400);
	Size imgSize = input.size();
	Point imgCenter( imgSize.width/2, imgSize.height/2 );
	for( size_t i = 0; i < circles.size() && i < 1; i++ ) {
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		int xDiff = imgCenter.x - center.x;
		int yDiff = imgCenter.y - center.y;
		char sendBuf;
		if( abs( yDiff ) > radius ) {
			yDiff = yDiff > 0 ? 30 : 10;
		} else {
			yDiff = 20;
		}
		if( abs( xDiff ) > radius ) {
			xDiff = xDiff > 0 ? 3 : 1;
		} else {
			xDiff = 2;
		}
		if( xDiff != 2 || yDiff != 20 ) {
			sendBuf = xDiff + yDiff;
			write( serial, &sendBuf, 1 );
		}
		rectangle(
			input,
			Point(
				circles[i][0] - radius,
				circles[i][1] - radius
			),
			Point(
				circles[i][0] + radius,
				circles[i][1] + radius
			),
			Scalar( 0, 255, 0 ),
			3,
			8,
			0
		);
		line( input, imgCenter, center, Scalar( 255, 0, 0 ), 3, 8, 0 );
	}
}



/**
 *  buffer header callback function for video
 *
 * @param port Pointer to port from which callback originated
 * @param buffer mmal buffer header pointer
 */
static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   MMAL_BUFFER_HEADER_T *new_buffer;
   PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;
   // init windows and OpenCV Stuff
   Mat origImage, threshedImage, greyImg, new_image;

   if (pData)
   {
      if (buffer->length)
      {
        mmal_buffer_header_mem_lock(buffer);

		//
		// *** PR : OPEN CV Stuff here !
		//
		int w=pData->pstate->width;    // get image size
		int h=pData->pstate->height;
		int h4=h/4;

		memcpy(py->imageData,buffer->data,w*h);    // read Y

		if (pData->pstate->graymode==0)
		{
			memcpy(pu->imageData,buffer->data+w*h,w*h4); // read U
			memcpy(pv->imageData,buffer->data+w*h+w*h4,w*h4); // read v

			cvResize(pu, pu_big, CV_INTER_NN);
			cvResize(pv, pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
			cvMerge(py, pu_big, pv_big, NULL, image);

			cvCvtColor(image,dstImage,CV_YCrCb2RGB);    // convert in RGB color space (slow)
			origImage=cvarrToMat(dstImage);
			//cvShowImage("camcvWin", dstImage );
			//cvWaitKey(1);
		}
		else
		{
			// for face reco, we just keep gray channel, py
			origImage=cvarrToMat(py);
			//cvShowImage("camcvWin", py); // display only gray channel
			//cvWaitKey(1);
		}
		cvtColor(origImage, greyImg, CV_RGB2GRAY);
		
		//Detect symbols
		Mat canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		GaussianBlur(greyImg, greyImg, Size(9, 9), 2, 2);

		/// Detect edges using canny
		Canny(greyImg, canny_output, lowThreshold, lowThreshold * 3, 3);

		//	imshow("B",canny_output);
		/// Find contours
		findContours(canny_output, contours, hierarchy, CV_RETR_TREE,
				CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		vector<Point> approxRect;

		for (size_t i = 0; i < contours.size(); i++) {
			approxPolyDP(contours[i], approxRect,
					arcLength(Mat(contours[i]), true) * 0.05, true);
			if (approxRect.size() == 4) {
				float area = contourArea(contours[i]);

				if (area > 10000) {
					std::vector<cv::Point2f> corners;

					vector<Point>::iterator vertex;
					vertex = approxRect.begin();
					//vertex++;
					circle(origImage, *vertex, 2, Scalar(0, 0, 255), -1, 8, 0);
					corners.push_back(*vertex);
					vertex++;
					circle(origImage, *vertex, 2, Scalar(0, 0, 255), -1, 8, 0);
					corners.push_back(*vertex);
					vertex++;
					circle(origImage, *vertex, 2, Scalar(0, 0, 255), -1, 8, 0);
					corners.push_back(*vertex);
					vertex++;
					circle(origImage, *vertex, 2, Scalar(0, 0, 255), -1, 8, 0);
					corners.push_back(*vertex);

					Moments mu;
					mu = moments(contours[i], false);
					Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);

					sortCorners(corners, center);

					// Define the destination image
					Mat correctedImg = ::Mat::zeros(195, 271, CV_8UC3);

					// Corners of the destination image
					std::vector<cv::Point2f> quad_pts;
					quad_pts.push_back(Point2f(0, 0));
					quad_pts.push_back(Point2f(correctedImg.cols, 0));
					quad_pts.push_back(
							Point2f(correctedImg.cols, correctedImg.rows));
					quad_pts.push_back(Point2f(0, correctedImg.rows));

					// Get transformation matrix
					Mat transmtx = getPerspectiveTransform(corners, quad_pts);

					// Apply perspective transformation
					warpPerspective(origImage, correctedImg, transmtx,
							correctedImg.size());

					Mat correctedImgBin;

					cvtColor(correctedImg, correctedImgBin, CV_RGB2GRAY);

					//equalizeHist(correctedImgBin, correctedImgBin);

					correctedImgBin.copyTo(new_image);

					threshold(correctedImgBin, correctedImgBin, 140, 255, 0);

					imshow("B", correctedImgBin);

					double minVal,maxVal,medVal;

					minMaxLoc(new_image, &minVal, &maxVal);

					medVal=(maxVal-minVal)/2;

					threshold(new_image, new_image, medVal, 255, 0);

					imshow("C", new_image);


					Mat diffImg;

					int match, minDiff, diff;
					minDiff = 12000;
					match = -1;

					for (int i = 0; i < 8; i++) {
						//diffImg = symbols[i].img-correctedImgBin;
						bitwise_xor(new_image, symbols[i].img, diffImg, noArray());

						diff = countNonZero(diffImg);
						if (diff < minDiff) {
							minDiff = diff;
							match = i;
						}

						if (i == 0) {
							//	imshow("B",diffImg);
						}
					}

					//imshow("B", correctedImg);

					if (match != -1) {
						putText(origImage, symbols[match].name, Point(320, 30), 1,
								2, Scalar(0, 255, 0), 2);
						//char sTmp[256];
						//sprintf (sTmp, "Symbol %s\n", symbols[match].name.c_str());
						//trace((string)(sTmp));
					}
					//break;
				}

			}
		}		
		//End detect symbols
		
		//Detect ball
		//int serial = 0;
		//threshedImage = Mat::zeros( origImage.size(), CV_8U );
	    //threshImage( origImage, threshedImage, serial );
		//imshow( "thresh", threshedImage );
		//End detect ball
		
		// Show the result:
		imshow("orig", origImage);		
		imshow( "gray", greyImg );		
		key = (char) waitKey(1);
		nCount++;    // count frames displayed

         mmal_buffer_header_mem_unlock(buffer);
      }
      else 
	  {
		  vcos_log_error("buffer null");
	  }
   }
   else
   {
      vcos_log_error("Received a encoder buffer callback with no state");
   }

   // release buffer back to the pool
   mmal_buffer_header_release(buffer);

   // and send one back to the port (if still open)
   if (port->is_enabled)
   {
      MMAL_STATUS_T status;

      new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);

      if (new_buffer)
         status = mmal_port_send_buffer(port, new_buffer);

      if (!new_buffer || status != MMAL_SUCCESS)
         vcos_log_error("Unable to return a buffer to the encoder port");
   }

}


/**
 * Create the camera component, set up its ports
 *
 * @param state Pointer to state control struct
 *
 * @return 0 if failed, pointer to component if successful
 *
 */
static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *preview_port = NULL, *video_port = NULL, *still_port = NULL;
    MMAL_STATUS_T status;

    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

    if (status != MMAL_SUCCESS)
    {
       vcos_log_error("Failed to create camera component");
       goto error;
    }

    if (!camera->output_num)
    {
       vcos_log_error("Camera doesn't have output ports");
       goto error;
    }

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    //  set up the camera configuration
    {
       MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
       {
          { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
          cam_config.max_stills_w = state->width,
          cam_config.max_stills_h = state->height,
          cam_config.stills_yuv422 = 0,
          cam_config.one_shot_stills = 0,
          cam_config.max_preview_video_w = state->width,
          cam_config.max_preview_video_h = state->height,
          cam_config.num_preview_video_frames = 3,
          cam_config.stills_capture_circular_buffer_height = 0,
          cam_config.fast_preview_resume = 0,
          cam_config.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
       };
       mmal_port_parameter_set(camera->control, &cam_config.hdr);
    }
    // Set the encode format on the video  port

    format = video_port->format;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->encoding = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = state->framerate;
    format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

    status = mmal_port_format_commit(video_port);
    if (status)
    {
       vcos_log_error("camera video format couldn't be set");
       goto error;
    }

    // PR : plug the callback to the video port
    status = mmal_port_enable(video_port, video_buffer_callback);
    if (status)
    {
       vcos_log_error("camera video callback2 error");
       goto error;
    }

   // Ensure there are enough buffers to avoid dropping frames
   if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
      video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;


   // Set the encode format on the still  port
   format = still_port->format;
   format->encoding = MMAL_ENCODING_OPAQUE;
   format->encoding_variant = MMAL_ENCODING_I420;
   format->es->video.width = state->width;
   format->es->video.height = state->height;
   format->es->video.crop.x = 0;
   format->es->video.crop.y = 0;
   format->es->video.crop.width = state->width;
   format->es->video.crop.height = state->height;
   format->es->video.frame_rate.num = 1;
   format->es->video.frame_rate.den = 1;

   status = mmal_port_format_commit(still_port);
   if (status)
   {
      vcos_log_error("camera still format couldn't be set");
      goto error;
   }


    //PR : create pool of message on video port
    MMAL_POOL_T *pool;
    video_port->buffer_size = video_port->buffer_size_recommended;
    video_port->buffer_num = video_port->buffer_num_recommended;
    pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
    if (!pool)
    {
       vcos_log_error("Failed to create buffer header pool for video output port");
    }
    state->video_pool = pool;

    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
       still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;

    /* Enable component */
    status = mmal_component_enable(camera);

    if (status)
    {
       vcos_log_error("camera component couldn't be enabled");
       goto error;
    }

    raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

    state->camera_component = camera;

    return camera;

error:

   if (camera)
      mmal_component_destroy(camera);

   return 0;
}

/**
 * Destroy the camera component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_camera_component(RASPIVID_STATE *state)
{
   if (state->camera_component)
   {
      mmal_component_destroy(state->camera_component);
      state->camera_component = NULL;
   }
}


/**
 * Destroy the encoder component
 *
 * @param state Pointer to state control struct
 *
 */
static void destroy_encoder_component(RASPIVID_STATE *state)
{
   // Get rid of any port buffers first
   if (state->video_pool)
   {
      mmal_port_pool_destroy(state->encoder_component->output[0], state->video_pool);
   }

   if (state->encoder_component)
   {
      mmal_component_destroy(state->encoder_component);
      state->encoder_component = NULL;
   }
}

/**
 * Connect two specific ports together
 *
 * @param output_port Pointer the output port
 * @param input_port Pointer the input port
 * @param Pointer to a mmal connection pointer, reassigned if function successful
 * @return Returns a MMAL_STATUS_T giving result of operation
 *
 */
static MMAL_STATUS_T connect_ports(MMAL_PORT_T *output_port, MMAL_PORT_T *input_port, MMAL_CONNECTION_T **connection)
{
   MMAL_STATUS_T status;

   status =  mmal_connection_create(connection, output_port, input_port, MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);

   if (status == MMAL_SUCCESS)
   {
      status =  mmal_connection_enable(*connection);
      if (status != MMAL_SUCCESS)
         mmal_connection_destroy(*connection);
   }

   return status;
}

/**
 * Checks if specified port is valid and enabled, then disables it
 *
 * @param port  Pointer the port
 *
 */
static void check_disable_port(MMAL_PORT_T *port)
{
   if (port && port->is_enabled)
      mmal_port_disable(port);
}

/**
 * Handler for sigint signals
 *
 * @param signal_number ID of incoming signal.
 *
 */
static void signal_handler(int signal_number)
{
  // Going to abort on all signals
  vcos_log_error("Aborting program\n");

  shouldExit = true;
}

/**
 * main
 */
int main(int argc, const char **argv)
{	
	if (readRefImages(symbols) == -1) {
		trace("Error reading reference symbols\n");
		return -1;
	}
	else
	{
		trace("Successful reading reference symbols\n");
	}
	
	lowThreshold = 50;
	
	//createTrackbar("Min Threshold:", "A", &lowThreshold, 100, CannyThreshold);
	
    // Our main data storage vessel..
    RASPIVID_STATE state;

    MMAL_STATUS_T status;// = -1;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;
    MMAL_PORT_T *preview_input_port = NULL;
    MMAL_PORT_T *encoder_input_port = NULL;
    MMAL_PORT_T *encoder_output_port = NULL;

    time_t timer_begin,timer_end;
    double secondsElapsed;

    bcm_host_init();
    signal(SIGINT, signal_handler);

    // read default status
    default_status(&state);

    

    cvNamedWindow("orig", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("thresh", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("gray", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("B", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("C", CV_WINDOW_AUTOSIZE);
	trace("CV_WINDOW_AUTOSIZE : ok");
    int w=state.width;
    int h=state.height;
    dstImage = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
    py = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);    // Y component of YUV I420 frame
    pu = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);    // U component of YUV I420 frame
    pv = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);    // V component of YUV I420 frame
    pu_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    pv_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    image = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);    // final picture to display

    state.camera_parameters.sharpness  = 50;
    state.camera_parameters.contrast   = 50;
    state.camera_parameters.brightness = 50;
    state.camera_parameters.saturation = 50;
    state.camera_parameters.exposureMode = MMAL_PARAM_EXPOSUREMODE_NIGHT;

    // create camera
    if (!create_camera_component(&state))
    {
       vcos_log_error("%s: Failed to create camera component", __func__);
    }
    //else if (!raspipreview_create(&state.preview_parameters))
    else if ( (status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
    {
       vcos_log_error("%s: Failed to create preview component", __func__);
       destroy_camera_component(&state);
    }
    else
    {
    PORT_USERDATA callback_data;

    camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
    camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];

    VCOS_STATUS_T vcos_status;

    callback_data.pstate = &state;

    vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
    vcos_assert(vcos_status == VCOS_SUCCESS);

    // assign data to use for callback
    camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

    // init timer
    time(&timer_begin);
	
    // start capture
    if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
    {
        return 0;
    }

    // Send all the buffers to the video port

    int num = mmal_queue_length(state.video_pool->queue);
    int q;
    for (q=0;q<num;q++)
    {
       MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);

       if (!buffer)
           vcos_log_error("Unable to get a required buffer %d from pool queue", q);

		if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
				vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
		}


    // Now wait until we need to stop
                while (!shouldExit) {
                  vcos_sleep(state.timeout);
                }

                //mmal_status_to_int(status);
                // Disable all our ports that are not handled by connections
                check_disable_port(camera_still_port);

                if (state.camera_component)
                    mmal_component_disable(state.camera_component);

                //destroy_encoder_component(&state);
                raspipreview_destroy(&state.preview_parameters);
                destroy_camera_component(&state);

                if (status != 0)
                  raspicamcontrol_check_configuration(128);

                time(&timer_end);  /* get current time; same as: timer = time(NULL)  */
                cvReleaseImage(&dstImage);
                cvReleaseImage(&pu);
                cvReleaseImage(&pv);
                cvReleaseImage(&py);
                cvReleaseImage(&pu_big);
                cvReleaseImage(&pv_big);

        }

        secondsElapsed = difftime(timer_end,timer_begin);

        char sTmp[128];
        sprintf (sTmp, "%.f seconds for %d frames : FPS = %f\n", secondsElapsed,nCount,(float)((float)(nCount)/secondsElapsed));
        trace((string)(sTmp));

        sprintf (sTmp, "Total Visitors: %d", totalVisitors);
        trace((string)(sTmp));

   return 0;
}

