#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using namespace std;
using namespace cv;

void threshImage( Mat input, Mat output, int serial ) {
	Mat hsvImg = Mat::zeros( input.size(), input.type() );
	vector<Vec3f> circles;

	blur( input, hsvImg, Size( 9, 9 ) );
	cvtColor( hsvImg, hsvImg, CV_BGR2HSV );
	inRange( hsvImg, Scalar( 70, 30, 30 ), Scalar( 100, 255, 255 ), output );

	HoughCircles( output, circles, CV_HOUGH_GRADIENT, 2, output.rows/4, 100, 40, 20, 200);
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

int openSerial( const char *path ) {
	int port;
	struct termios options;

	port = open( path, O_RDWR | O_NOCTTY | O_NDELAY );
	if( port == -1 ) {
		printf( "Unable to open port %s\n", path );
		return -1;
	}
	fcntl( port, F_SETFL, 0 );

	tcgetattr( port, &options );
	cfsetispeed( &options, B115200 );
	cfsetospeed( &options, B115200 );
	tcsetattr( port, TCSANOW, &options );

	return port;
}

int main( int argc, char **argv ) {
	CvCapture *capture;
	Mat origImage, threshedImage;
	int serial;

	if( argc < 2 ) {
		cerr << "Usage: " << argv[0] << " <serial_port>" << endl;
		return -1;
	}

	serial = openSerial( argv[1] );
	if( serial == -1 ) {
		return -1;
	}
	cvNamedWindow( "orig" );
	cvNamedWindow( "thresh" );
	capture = cvCaptureFromCAM( -1 );
	if( capture ) {
		while( true ) {
			origImage = cvQueryFrame( capture );
			if( origImage.empty() ) {
				cerr << "Failed to load image" << endl;
				return -1;
			}
			threshedImage = Mat::zeros( origImage.size(), CV_8U );
			threshImage( origImage, threshedImage, serial );

			imshow( "orig", origImage );
			imshow( "thresh", threshedImage );

			if( ( char )waitKey( 10 ) == 'c' ) {
				break;
			}
		}
	}


	close( serial );
	destroyAllWindows();
	return 0;
}