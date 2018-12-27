#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv_apps/RotatedRectArrayStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace cv;
using namespace std;

std_msgs::Int32MultiArray array;

//arr.data.clear();

 int iLowH = 31;//0;
 int iHighH = 159;//30;

 int iLowS = 18;//31; 
 int iHighS = 255;

 int iLowV = 128;//0;
 int iHighV = 255;

//cv::Size displaySize(1280, 720);
//bool temp[1280][720];
bool newImage;
Mat imgOriginal;
Mat imgMod;
//Mat imgPast;
Mat imgNew;
Point rectCenter;
//////////////////////////////////////////////////////////////////
ros::Publisher centerPoint_pub;
image_transport::Publisher binaryImagePublisher;
//////////////////////////////////////////////////////////////////
struct kalman_state{
  double q; //process noise covariance
  double r; //measurement noise covariance
  double x; //value
  double p; //estimation error covariance
  double k; //kalman gain
};

kalman_state kalman_init(double q, double r, double p, double intial_value)
{
  kalman_state result;
  result.q = q;
  result.r = r;
  result.p = p;
  result.x = intial_value;

  return result;
}

void kalman_update(kalman_state* state, double measurement)
{
  //prediction update
  //omit x = x
  state->p = state->p + state->q;

  //measurement update
  state->k = state->p / (state->p + state->r);
  state->x = state->x + state->k * (measurement - state->x);
  state->p = (1 - state->k) * state->p;
}
///////////////////////////////////////////////////////////////////
kalman_state k_state_x = kalman_init(7,5,0,640);
kalman_state k_state_y = kalman_init(7,5,0,360);

/////////////////////////////////////////////////////////////////////
void colorThresholdAndFill(Mat& imgIn, Mat& imgOut)
{
	Mat imgHSV;

  cvtColor(imgIn, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
 
  inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgOut); //Threshold the image
      
  //morphological opening (remove small objects from the foreground)
  erode(imgOut, imgOut, getStructuringElement(MORPH_RECT, Size(5, 5)) );
  dilate( imgOut, imgOut, getStructuringElement(MORPH_RECT, Size(5, 5)) ); 

  //morphological closing (fill small holes in the foreground)
  dilate( imgOut, imgOut, getStructuringElement(MORPH_RECT, Size(5, 5)) ); 
  erode(imgOut, imgOut, getStructuringElement(MORPH_RECT, Size(5, 5)) );
}
///////////////////////////////////////////////////////////////////
void addCircleToCenter(Mat& imgIn, Mat& imgOut, Point& center)
{
		//Moments mu = moments(imgIn, true); 

    //Point center; //center of the target
    //center.x = mu.m10 / mu.m00;
    //center.y = mu.m01 / mu.m00;

		kalman_update(&k_state_x, center.x);
		kalman_update(&k_state_y, center.y);

		center.x = k_state_x.x;
		center.y = k_state_y.x;

		k_state_x.q = 0.3; k_state_y.q = 0.3;
		k_state_x.r = 5; k_state_y.r = 5;

    //Mat3b res;
    //cvtColor(imgIn, imgIn, CV_GRAY2BGR);

    circle(imgOut, center, 2, Scalar(0,0,255));

		Point image_center; //center of the image
	 	image_center.x = imgOut.cols/2.0;

	 	image_center.y = imgOut.rows/2.0;

		//imshow("Control", imgOut); 
}
//////////////////////////////////////////////////////////////////
void addRectangleToBlob(Mat& imgIn, Mat& imgOut)
{
	if (countNonZero(imgIn) > 0)
	{
	Rect rectangle_D;
	vector<Point> locations;
	findNonZero(imgIn,locations);
	rectangle_D = boundingRect(locations);

	rectangle(imgOut, rectangle_D.tl(), rectangle_D.br(), Scalar(255, 0, 255), CV_FILLED, 8, 0);
	}
}
////////////////////////////////////////////////////////////////////
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	Point center;
	Mat imgThresholded;
	string s;
	s = msg->encoding;
	//ROS_INFO("%s", s.c_str());
	//cv::Mat imgOriginal;//(displaySize, CV_8UC4);

 	imgOriginal = cv_bridge::toCvShare(msg)->image;
	imgMod = imgOriginal.clone();
	newImage = 1;

//	imshow("Control", imgOriginal);
	dilate( imgOriginal, imgOriginal, getStructuringElement(MORPH_RECT, Size(15, 15)) );
	colorThresholdAndFill(imgOriginal, imgThresholded);
// 	imshow("Control", imgThresholded); //show the thresholded image

	//addCircleToCenter(imgThresholded, imgThresholded, center);
	//imshow("Control", imgThresholded);
}
////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
void depthCallback(const sensor_msgs::Image& msg)
{
Mat imgColorThresholded;
Mat imgDepthThresholded;
Mat img;
Mat imgPast;
unsigned short intensity = 0;
Point center;
cv_bridge::CvImage out_msg;


cv_bridge::CvImagePtr ptr;
ptr = cv_bridge::toCvCopy(msg,"16UC1");
img = ptr->image;

//ROS_INFO("Number of Rows %i",imgMod.size().height);
//ROS_INFO("Number of Cols %i",imgMod.size().width);
//ROS_INFO("Number of Channels %i",imgMod.channels());
//	string s = msg.encoding;
//	ROS_INFO("%s", s.c_str());

if(!imgMod.empty() && newImage)
{

	for (int y = 0; y < img.size().height; y++)
		for (int x = 0; x < img.size().width; x++)
		{
			intensity = img.at<unsigned short>(y, x);
			//ROS_INFO("%f",static_cast<float>(intensity.val[0]));
				if(intensity > 0.0 && intensity < 2500 )
				{
					imgMod.at<Vec3b>(y, x)[0] = 255;
					imgMod.at<Vec3b>(y, x)[1] = 255;
					imgMod.at<Vec3b>(y, x)[2] = 255;

				}
				else
				{
					imgMod.at<Vec3b>(y, x)[0] = 0;
					imgMod.at<Vec3b>(y, x)[1] = 0;
					imgMod.at<Vec3b>(y, x)[2] = 0;

				}
		}

	
	//inRange(img, 0.0, 20, imgDepthThresholded);
//	imshow("Original", imgMod);

	out_msg.header   = msg.header; // Same timestamp and tf frame as input image
	out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
	out_msg.image    = imgMod;//imgOriginal; // Your cv::Mat
	
	binaryImagePublisher.publish(out_msg.toImageMsg());

	addCircleToCenter(imgMod, imgMod, rectCenter);
//	imshow("Mod", imgMod);
	//dilate( imgMod, imgMod, getStructuringElement(MORPH_RECT, Size(5, 5)) );
	//erode( imgMod, imgMod, getStructuringElement(MORPH_RECT, Size(45, 45)) );
	
	//imshow("Original", imgMod); //show the thresholded image
	
	//colorThresholdAndFill(imgMod, imgPast); // OUTPUT SHOULD BE 1 CHANNEL

	//imshow("Original", imgPast); //show the thresholded image

	//bitwise_and(imgDepthThresholded, imgColorThresholded, imgPast);
	//imshow("Mod", imgTemp);
	
	//imshow("Mod", imgPast);
	//addRectangleToBlob(imgPast, imgPast);
	
	//imshow("Mod", imgPast);	 

	//if(countNonZero(imgPast) > 35000)
	//imgNew = imgPast;
	//else
	//ROS_INFO("Not Enough Depth Info");
	
	//if(!imgNew.empty())
	//{
	//addCircleToCenter(imgNew, imgNew, center);
//	imshow("Mod", imgNew);

	array.data.clear();
	array.data.push_back(img.size().height);
	array.data.push_back(img.size().width);
	array.data.push_back(rectCenter.x);
	array.data.push_back(rectCenter.y);

	centerPoint_pub.publish(array);
	//}
	
	newImage = 0;

}

/*
std::string s;
s = msg.encoding;
ROS_INFO("Rows %i", msg.height);
ROS_INFO("Columns %i", msg.width);
ROS_INFO("Step %i", msg.step);
ROS_INFO("Encoding %s", s.c_str());
ROS_INFO("First Byte %i", msg.data[0]);
ROS_INFO("Second Byte %i", msg.data[1]);
ROS_INFO("Third Byte %i", msg.data[2]);
ROS_INFO("Fourth Byte %i", msg.data[3]);
ROS_INFO("Fifth Byte %i", msg.data[4]);
ROS_INFO("Sixth Byte %i", msg.data[5]);
ROS_INFO("Seventh Byte %i", msg.data[6]);
ROS_INFO("Eighth Byte %i", msg.data[7]);
*/
}
void rectCallback(const opencv_apps::RotatedRectArrayStamped& msg)
{
//ROS_INFO("%i", msg.rects.size());
float temp = 0;
float x = 0; float y = 0;
//ROS_INFO("%i", msg.rects.size());

for (int i = 0; i < msg.rects.size(); i++)
{
if(msg.rects[i].size.width * msg.rects[i].size.height > temp)
{
temp = msg.rects[i].size.width * msg.rects[i].size.height;
x = msg.rects[i].center.x;
y = msg.rects[i].center.y;
}
}

//ROS_INFO(" Rectangle Area : %f", temp);
rectCenter.x = x;
rectCenter.y = y;

}
/////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{

  ros::init(argc, argv, "objectdetect_node");
  ros::NodeHandle nh;

	centerPoint_pub = nh.advertise<std_msgs::Int32MultiArray>("/centerPoint", 100);	
	
	
//	cv::namedWindow("Original");
//  cv::namedWindow("Control");
//	cv::namedWindow("Mod");
	 //Create trackbars in "Control" window
// 	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
// 	cvCreateTrackbar("HighH", "Control", &iHighH, 179);
// 	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
// 	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

//	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
//	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
//  cv::startWindowThread();


  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);
	ros::Subscriber sub3 = nh.subscribe("/camera/depth/image_raw", 1, depthCallback);
	ros::Subscriber sub4 = nh.subscribe("/general_contours/rectangles", 1, rectCallback);

	//image_transport::Subscriber sub_1 = it.subscribe("camera/depth/image_rect_color", 1, depthCallback);
	binaryImagePublisher = it.advertise("/image", 1);
	

  ros::spin();
//	cv::destroyWindow("Original");
//  cv::destroyWindow("Control");
//	cv::destroyWindow("Mod");
}

