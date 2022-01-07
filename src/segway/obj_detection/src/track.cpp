#if 1


#include <sstream>
#include <random>
#include <string>
#include <iostream>
#include <time.h>


// #include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
// #include <opencv/highgui.h>
#include <opencv2/highgui.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>


#include <ros/ros.h>
#include <ros/types.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32MultiArray.h>


using namespace cv;
using namespace std;
namespace enc = sensor_msgs::image_encodings;


// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
( std::ostringstream() << std::dec << x ) ).str()

//#define MAX_TRACKING 10
#define MAX_TRACKING 1

Rect2d bbox(0, 0, 0, 0);
bool detection_start;
bool new_bbox_arrived = false; 
Mat frame; 
Mat gray;
Mat frame_depth;

cv_bridge::CvImageConstPtr cv_ptr_rgb;
cv_bridge::CvImageConstPtr cv_ptr_depth;

enum Mode {IDLE, WAITING, TRACK} mode;
Mat frame_static;
std::default_random_engine generator;
std::normal_distribution<double> distribution(5, 1.0);
Ptr<TrackerKCF> tracker;

void targetbbox_cb(const std_msgs::Int32MultiArray::ConstPtr& msg){
    int x = msg->data[0];
    int y = msg->data[1];
    int width = msg->data[2];
    int height = msg->data[3];
    bbox = Rect2d(x, y, width, height);
    new_bbox_arrived = true;
}

void rgb_cb(const sensor_msgs::Image::ConstPtr& msg){
    try{
        cv_ptr_rgb = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    static int i = 0;
    frame = cv_ptr_rgb->image;
}

void depth_cb(const sensor_msgs::Image::ConstPtr& msg){
    try{
        cv_ptr_depth = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    frame_depth = cv_ptr_depth->image;
}


float calc_depth(){

    int dims = 1; // Depth based
    bool uniform = true, accumulate = false;

    double mindepth, maxdepth;
    cv::minMaxLoc(frame_depth, &mindepth, &maxdepth);


    int histSize = 256;
    //float range[] = { 0, 256 }; //the upper boundary is exclusive
    float range[] = {mindepth,maxdepth};
    float delRange = (maxdepth-mindepth)/histSize;

    // calc hist of bboxed frame_depth
    const float* histRange = { range };


    //cv::Mat roi = frame_depth.clone();
    cv::Mat roi = frame_depth(bbox);
    //rect_side = rect_side(bbox);
    cv::Mat depthHist;
    //calcHist(&frame_depth, 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    calcHist(&roi, 1, 0, Mat(), depthHist, 1, &histSize, &histRange, uniform, accumulate );
    float peak;
    int peakhist = 0;
    for( int i = 0; i < histSize; i++ )
    {
        if (depthHist.at<float>(i) > peakhist){
            peakhist = depthHist.at<float>(i);
            peak = i*delRange+mindepth;
        }

    }
    return peak;
    
}

float calc_x(){
    return bbox.x + bbox.width / 2;
}

float calc_y(){
    return bbox.y + bbox.height / 2;
}

bool updateBbox(Rect2d& bbox){
    ros::Duration(0.05).sleep(); // sleep for half a second
    bbox.x += distribution(generator);
    bbox.y += distribution(generator);
    bbox.width = distribution(generator);
    bbox.height = distribution(generator);
    return true;
}

void sobelExtractor(const Mat img, const Rect roi, Mat& feat);
int main(int argc, char **argv)
{


    ROS_INFO("%d", CV_MINOR_VERSION);
    ROS_INFO("%d", CV_MAJOR_VERSION);

    string trackerTypes[8] = {"BOOSTING", "MIL", "KCF", "TLD","MEDIANFLOW", "GOTURN", "MOSSE", "CSRT"};
    string trackerType = trackerTypes[2];
    
    int tracking_count = 0;

    ros::init(argc, argv, "tracker");
    ros::NodeHandle n;
    ros::Publisher targetd_pub = n.advertise<std_msgs::Float64>("/targetD", 1);
    ros::Publisher targetx_pub = n.advertise<std_msgs::Float64>("/targetX", 1);
    ros::Publisher targety_pub = n.advertise<std_msgs::Float64>("/targetY", 1);
    ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>("/rgb_with_dets", 1);
    ros::Subscriber targetbbox_sub = n.subscribe("/bounding_box", 1000, targetbbox_cb);
    ros::Subscriber image_sub = n.subscribe("/zed/color/image_raw", 1, rgb_cb);
    ros::Subscriber depth_sub = n.subscribe("/zed/depth/image_raw", 1, depth_cb);
    ros::Publisher detection_starter_pub = n.advertise<std_msgs::Empty>("/start_detection", 1, true);

   TrackerKCF::Params param;
   param.desc_pca = TrackerKCF::GRAY | TrackerKCF::CN;
   param.desc_npca = 0;
   param.compress_feature = true;
   param.compressed_size = 2;
 
    tracker = TrackerKCF::create(param);
    // Read first frame 
    tracker->setFeatureExtractor(sobelExtractor);
    // Uncomment the line below to select a different bounding box 
    // bbox = selectROI(frame, false); 
    // Display bounding box. 
    
    
    std_msgs::Float64 x_msg, y_msg, depth_msg;
    int depth;
    float x, y;
    std_msgs::Empty signal;
    ros::Duration(10).sleep(); // sleep for half a second
    Rect bbox_initial = bbox;

    while(true){
        ros::spinOnce();
        switch (mode){


            case IDLE:
            {
                detection_starter_pub.publish(signal);
                mode = WAITING;
                ROS_INFO("Mode changed to WAITING");
                break;
            }
            case WAITING:
            {
                if (new_bbox_arrived){
                    new_bbox_arrived = false;
                    
                    //quit if ROI was not selected
                      if(bbox.width==0 || bbox.height==0){
                          ROS_INFO("Cannot initialize - ROI empty!");
                          break;
                    }

                    tracker->init(frame, bbox);

                    mode = TRACK;
                    ROS_INFO("Mode changed to TRACK");
                }
                break;
            }
            case TRACK:
        {
            //quit if ROI was not selected
            if(bbox.width==0 || bbox.height==0){
                ROS_INFO("Cannot update - ROI empty!");
                        break;
            }


            if(frame.rows==0 || frame.cols==0){
                ROS_INFO("Cannot update - Frame empty!");
                        break;
            }

            bool ok = tracker->update(frame, bbox);
            // bool ok = updateBbox(bbox);
            if (ok){
                tracking_count++;
                    ROS_INFO("Tracking the target at [%d, %d, %d, %d] (x, y, w, h)", (long int)bbox.x,  (long int)bbox.y,  (long int)bbox.width,  (long int)bbox.height);
                    rectangle(frame, bbox, Scalar( 255, 0, 0 ), 2, 1 ); 
                    depth_msg.data = calc_depth();
                    //depth_msg.data = 5;
                    ROS_INFO("Target depth is  [%f] m", depth_msg.data);
                    x_msg.data = calc_x();
                    y_msg.data = calc_y();

                    targetd_pub.publish(depth_msg);
                    rgb_pub.publish(cv_ptr_rgb->toImageMsg());
                    targetx_pub.publish(x_msg);
                    targety_pub.publish(y_msg);
                    if (tracking_count == MAX_TRACKING){
                        mode = IDLE;
                        tracking_count = 0;
                        ROS_INFO("Mode changed to IDLE");
                    }
                }
                else{
                    ROS_INFO("Tracking failure.");
                }
                break;
            }
            default:
            {
                ROS_INFO("Tracker invalid mode.");
            }

        }


    }
}
 void sobelExtractor(const Mat img, const Rect roi, Mat& feat){
     Mat sobel[2];
     Mat patch;
     Rect region=roi;
 
     // extract patch inside the image
     if(roi.x<0){region.x=0;region.width+=roi.x;}
     if(roi.y<0){region.y=0;region.height+=roi.y;}
     if(roi.x+roi.width>img.cols)region.width=img.cols-roi.x;
     if(roi.y+roi.height>img.rows)region.height=img.rows-roi.y;
     if(region.width>img.cols)region.width=img.cols;
     if(region.height>img.rows)region.height=img.rows;
 
     patch=img(region).clone();
     cvtColor(patch,patch, COLOR_BGR2GRAY);
 
     // add some padding to compensate when the patch is outside image border
     int addTop,addBottom, addLeft, addRight;
     addTop=region.y-roi.y;
     addBottom=(roi.height+roi.y>img.rows?roi.height+roi.y-img.rows:0);
     addLeft=region.x-roi.x;
     addRight=(roi.width+roi.x>img.cols?roi.width+roi.x-img.cols:0);
 
     copyMakeBorder(patch,patch,addTop,addBottom,addLeft,addRight,BORDER_REPLICATE);
 
     Sobel(patch, sobel[0], CV_32F,1,0,1);
     Sobel(patch, sobel[1], CV_32F,0,1,1);
 
     merge(sobel,2,feat);
 
     feat=feat/255.0-0.5; // normalize to range -0.5 .. 0.5
 }
#endif
#if 0
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include "ros/ros.h"
using namespace std;
using namespace cv;
int main( int argc, char** argv ){
  // show help
  if(argc<2){
    cout<<
      " Usage: tracker <video_name>\n"
      " examples:\n"
      " example_tracking_kcf Bolt/img/%04d.jpg\n"
      " example_tracking_kcf faceocc2.webm\n"
      << endl;
    return 0;
  }
  // declares all required variables
  Rect roi;
  Mat frame;
  // create a tracker object
  Ptr<Tracker> tracker = TrackerMIL::create();
  // set input video
  std::string video = argv[1];
  VideoCapture cap(video);
  // get bounding box
  cap >> frame;
  roi=selectROI("tracker",frame);
  //quit if ROI was not selected
  if(roi.width==0 || roi.height==0)
    return 0;
  // initialize the tracker
  tracker->init(frame,roi);
  // perform the tracking process
  printf("Start the tracking process, press ESC to quit.\n");
  for ( ;; ){
    // get frame from the video
    cap >> frame;
    // stop the program if no more images
    if(frame.rows==0 || frame.cols==0)
      break;
    // update the tracking result
    tracker->update(frame,roi);
    // draw the tracked object
    rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );
    // show image with the tracked object
    imshow("tracker",frame);
    //quit on ESC button
    if(waitKey(1)==27)break;
  }
  return 0;
}

#endif
