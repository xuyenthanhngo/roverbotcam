// FaceDetect1.cpp
//
//		Display video from webcam and detect faces
//

#include "stdafx.h"
#include <stdio.h>
#include "cv.h"
#include "highgui.h"
#include <string.h>

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
	int       key = NULL;
	char      *filename = "haarcascade_frontalface_alt.xml";

	cascade = ( CvHaarClassifierCascade* )cvLoad( filename, 0, 0, 0 );
	storage = cvCreateMemStorage( 0 );
	capture = cvCaptureFromCAM( 0 );

	assert( cascade && storage && capture );

	cvNamedWindow( "video", 1 );

	while( key != 27 ) {
		frame = cvQueryFrame( capture );

		if( !frame ) {
			fprintf( stderr, "Cannot query frame!\n" );
			break;
		}

		// get properties, needed to create rotated image
		int width     = frame->width;
		int height    = frame->height;
		int depth     = frame->depth;
		int nchannels = frame->nChannels;

//		printf( "width:  %i \nheight: %i\n", width,height );

		// create destination image with transposed height and width values
		rotated = cvCreateImage( cvSize( height, width ), depth, nchannels );

		cvTranspose( frame, rotated );

		rotated->origin = 0;

		coords mycoords = detectFaces( rotated );

		printf( "x:  %3i\t y:  %3i\n", mycoords.x,mycoords.y );

		key = cvWaitKey( 10 );

		cvReleaseImage( &rotated );
	}

	cvReleaseCapture( &capture );
	cvDestroyWindow( "video" );
	cvReleaseHaarClassifierCascade( &cascade );
	cvReleaseMemStorage( &storage );

	return 0;
}

coords detectFaces( IplImage *img )
{
//	int i;
	coords center;
	center.x = -1;
	center.y = -1;

	CvSeq *faces = cvHaarDetectObjects(
			img,
			cascade,
			storage,
			1.1,
			3,
//`			0 /*CV_HAAR_DO_CANNY_PRUNNING*/,
			CV_HAAR_FIND_BIGGEST_OBJECT,
			cvSize( 40, 40 ) );

	if( faces && faces->total > 0 ) {
//	for( i = 0 ; i < ( faces ? faces->total : 0 ) ; i++ ) {
//		CvRect *r = ( CvRect* )cvGetSeqElem( faces, i );
		CvRect *r = ( CvRect* )cvGetSeqElem( faces, 0 ); 
		
		center.x = r->x + r->width/2;
		center.y = r->y + r->height/2;

		cvRectangle( img,
					 cvPoint( r->x, r->y ),
					 cvPoint( r->x + r->width, r->y + r->height ),
					 CV_RGB( 255, 0, 0 ), 1, 8, 0 );
		
	}

//	printf( "x:  %i \ny:  %i\n", x,y );

	cvShowImage( "video", img );

	return center;
}