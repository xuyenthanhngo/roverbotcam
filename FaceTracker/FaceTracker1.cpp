// FaceTracker1.cpp
//
//		Processes video from a webcam to detect the largest (closest) human face.
//	Serial commands sent out a COM port to an ATmega328P microcontroller to control
//	the camera's pan and tilt servos, keeping the detected face centered in the video.
//
//	Notes:	Use this program in conjunction with "servoControl3.c" microcontroller firmware.
//
//	Author(s):	Jeremy Greenwood, David Kelly


#include "stdafx.h"
#include <stdio.h>
#include <math.h>
#include "cv.h"
#include "highgui.h"
#include <string.h>
#include "SerialPort.h"
using namespace std;

#define METHOD	2
#define DEV		4			// define off-center deviation parameter
#define MULT	0.95		// define multiplier for number of commands
#define CUT		14.0 		// define cut, the number to decrease overall serial commands sent

CvHaarClassifierCascade *cascade;
CvMemStorage            *storage;

typedef struct _coords
{
	int x;
	int y;
} coords;

coords detectFaces( IplImage *img );

int _tmain(int argc, _TCHAR* argv[])
{
	CvCapture *capture;
	IplImage  *frame, *rotated;
	int       key = NULL, width, height, depth, nchannels, x_origin, y_origin, x_diff, y_diff,
				x_count, y_count, max_count, temp1, temp2;
	char      *filename = "haarcascade_frontalface_alt.xml", *x_dir = NULL, *y_dir = NULL;

	cascade = ( CvHaarClassifierCascade* )cvLoad( filename, 0, 0, 0 );
	storage = cvCreateMemStorage( 0 );
	capture = cvCaptureFromCAM( 0 );

	assert( cascade && storage && capture );

	cvNamedWindow( "video", 1 );
	
	CSerialPort serial;	
	serial.OpenSerial();			// open a serial connection (configured in SerialPort.cpp)

	while( key != 27 ) {			// escape key kills program
		frame = cvQueryFrame( capture );

		if( !frame ) {
			fprintf( stderr, "Cannot query frame!\n" );
			break;
		}

		// get properties, needed to create rotated image
		width     = frame->width;
		height    = frame->height;
		depth     = frame->depth;
		nchannels = frame->nChannels;

		// create destination image with transposed height and width values
		rotated = cvCreateImage( cvSize( height, width ), depth, nchannels );
		cvTranspose( frame, rotated );

		rotated->origin = 0;

		coords face_coord = detectFaces( rotated );

		// send serial commands to microcontroller to alter pan and tilt motor positions:
		// 4-left, 6-right, 8-up, 2-down
		if( face_coord.x != -1 ) {
			x_origin = height / 2;		// Note: height is actually width and vice versa due to camera rotation
			y_origin = width / 2;

			x_diff = face_coord.x - x_origin;
			y_diff = face_coord.y - y_origin;

			printf( "x:  %3i\t\t y:  %3i\n", x_diff, y_diff );		// display difference of centroid coordinates

			// Tracking method 1:  simple algorithm which sends more move instructions the larger x_diff and y_diff become
			if( METHOD == 1 ) {
				if( x_diff > DEV )
					for( int i = 0; i < (double)x_diff * MULT - CUT; i++ )
						serial.Send("4");

				if( x_diff < -DEV )
					for( int i = 0; i <  (double)-x_diff * MULT - CUT; i++ )
						serial.Send("6");

				if( y_diff > DEV )
					for( int i = 0; i <  (double)y_diff * MULT - CUT; i++ )
						serial.Send("2");

				if( y_diff < -DEV )
					for( int i = 0; i <  (double)-y_diff * MULT - CUT; i++ )
						serial.Send("8");
			}

			// Tracking method 2:	same as method 1 but employs a technique to smooth diagonal scrolling
			//						even for non 1:1 diagonal movement
			if( METHOD == 2 ) {

				if( x_diff > DEV )
					x_dir = "4";
				else if( x_diff < -DEV )
					x_dir = "6";

				if( y_diff > DEV )
					y_dir = "2";
				else if( y_diff < -DEV )
					y_dir = "8";

				if( y_dir && x_dir ) {		// this is the case where diagonal scrolling occurs

					x_count = (double)abs(x_diff) * MULT - CUT;
					y_count = (double)abs(y_diff) * MULT - CUT;
					max_count = max(x_count, y_count);

					for( int i = 0; i < max_count; i++ ) {		// alternate between x and y movements

						temp1 = (int)(x_count / max_count);
						temp2 = (double)x_count / (double)max_count;
//						printf( "temp1x: %3i\ttemp2x: %d\n", temp1, temp2 );		// debugging message

						if( temp1 == temp2 )
							serial.Send( x_dir );

						temp1 = (int)(y_count / max_count);
						temp2 = (double)y_count / (double)y_count;
//						printf( "temp1y: %3i\ttemp2y: %d\n", temp1, temp2 );		// debugging message

						if( temp1 == temp2 )
							serial.Send( y_dir );
					}
					x_dir = NULL;
					y_dir = NULL;
				}
				else if( x_dir ) {

					x_count = (double)abs(x_diff) * MULT - CUT;

					for( int i = 0; i < x_count; i++ )
						serial.Send( x_dir );
					x_dir = NULL;
				}
				else if( y_dir ) {

					y_count = (double)abs(y_diff) * MULT - CUT;

					for( int i = 0; i < y_count; i++ )
						serial.Send( y_dir );
					y_dir = NULL;
				}
			}
		}
		key = cvWaitKey( 1 );
		cvReleaseImage( &rotated );
	}
	serial.CloseSerial();
	cvReleaseCapture( &capture );
	cvDestroyWindow( "video" );
	cvReleaseHaarClassifierCascade( &cascade );
	cvReleaseMemStorage( &storage );

	return 0;
}

coords detectFaces( IplImage *img )
{
	coords face_centroid;
	face_centroid.x = -1;
	face_centroid.y = -1;

	CvSeq *faces = cvHaarDetectObjects(
			img,
			cascade,
			storage,
			1.1,
			3,
			CV_HAAR_FIND_BIGGEST_OBJECT,	// try to detect only the closest face 
			cvSize( 40, 40 ) );

	if( faces && faces->total > 0 ) {
		CvRect *r = ( CvRect* )cvGetSeqElem( faces, 0 ); 
		
		face_centroid.x = r->x + r->width/2;
		face_centroid.y = r->y + r->height/2;

		cvRectangle( img,
					 cvPoint( r->x, r->y ),
					 cvPoint( r->x + r->width, r->y + r->height ),
					 CV_RGB( 255, 0, 0 ), 1, 8, 0 );
	}
//	printf( "x:  %i \ny:  %i\n", x,y );
	cvShowImage( "video", img );

	return face_centroid;
}
