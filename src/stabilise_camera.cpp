// Node for stabilising an image.
// Helpful for visually inspecting the accuracy time synchronisation between a camera and an IMU.
//
// Subscribes to a ROS image topic and an IMU topoic and stabilises
// the image based on the gravity vector of the IMU.


// TODO: requrite as a class

#include <ros/ros.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

double scale = 1;
image_geometry::PinholeCameraModel camera_model;
bool output_average_frame;
bool fix_yaw;
bool find_horizon;
bool homography;
double scale_f;
bool chop;
bool filter_yaw;


double fx = 929.9355301091493 * 1;
double fy = 929.2498095046924 * 1;
double cx = 633.9987150324908;
double cy = 382.8913463987184;

double k1 = -0.1651660818025783;
double k2 = 0.09063217218909561 ;
double p1 = -0.0010560133963248134;
double p2 = -0.000204423321673326;
double k3 = 0.0;

void averageFrames(cv::Mat & image, cv::Mat &result)
{

  result.create(image.rows,image.cols,image.type());

  typedef int32_t MatType;

  static std::vector<cv::Mat_<MatType> > images;
  static cv::Mat_<MatType> average(image.rows,image.cols,0.0);
  cv::Mat_<MatType> image_temp; image.convertTo(image_temp, CV_MAKETYPE(CV_32S, image.channels()));

  static uint i = 0;

  if (images.size() < 10 ) {
    images.push_back(image_temp.clone());
    average = average + image_temp;
  }
  else
  {
    average = average + image_temp - images[i];
    images[i] = image_temp;
  }

  i++;
  if (i >= images.size())
    i =0;

  cv::Mat(average/images.size()).convertTo(result, image.type());

}

void undistortImage(cv::Mat & image, cv::Mat &result){


  result.create(image.rows,image.cols,image.type());
  float cm[] = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  cv::Mat cameraMatrix(3, 3, CV_32F, cm);
  cv::Mat map1, map2;

  float k[] = {k1, k2, p1, p2, k3};
  cv::Mat distCoeffs(1, 5, CV_32F, k);

  if (map1.empty() == true){
  cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, image.size(), CV_32FC1, map1, map2);
  }

  cv::remap(image, result, map1, map2, cv::INTER_LINEAR);

}


void stabaliseImageHomography(cv::Mat & image, cv::Mat &result, double roll, double pitch, double yaw)
{

  result.create(image.rows,image.cols,image.type());


  float Si[] = {fx*scale ,     0.0,  0,
               0.0 ,    fy*scale,  0,
               0.0 ,     0.0,  1.0
              };


  float S[] = {1.0/(fx*scale)  ,  0.0    ,  0,
               0.0      ,  1.0/(fy*scale) ,  0,
               0.0      ,  0.0    ,  1.0
              };


  float T[] = {1.0 ,     0.0,  -1*(cx*scale),
               0.0 ,     1.0,  -1*(cy*scale),
               0.0 ,     0.0,      1.0
              };


  float Ti[] = {1.0 ,     0.0,  (cx*scale),
               0.0 ,     1.0,   (cy*scale),
               0.0 ,     0.0,      1.0
              };

  float R[] = {cos(roll),     sin(roll),  0.0 ,
               -1*sin(roll),  cos(roll),  0.0,
               0.0      , 0.0          ,  1.0};

  float P[] = {1,          0,             0.0 ,
               0,       cos(pitch),    sin(pitch) ,
               0,      -1*sin(pitch),   cos(pitch)       };


  float Y[] = {1,          sin(yaw)-cos(yaw),    cos(yaw) + sin(yaw),
               0,       cos(pitch),    sin(pitch) ,
               0,      -1*sin(pitch),   cos(pitch)       };

  float alpha = -1 * roll;
  float beta  =  1 * yaw;
  float gamma = -1 * pitch;


  float A[] = {cos(alpha) * cos(beta),  cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma)   ,  cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
               sin(alpha) * cos(beta),  sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma)   ,  sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma),
      -1.0 * sin(beta),      cos(beta) * sin(gamma),   cos(beta) * cos(gamma)       };


  cv::Mat p(3, 3, CV_32F, P);
  cv::Mat r(3, 3, CV_32F, R);
  cv::Mat t(3, 3, CV_32F, T);
  cv::Mat ti(3, 3, CV_32F, Ti);
  cv::Mat a(3, 3, CV_32F, A);
  cv::Mat s(3, 3, CV_32F, S);
  cv::Mat si(3, 3, CV_32F, Si);

  cv::Mat H = ti*si*a*s*t;

  if (find_horizon)
  {
    cv::warpPerspective(image,result,H,image.size(),  cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,cv::BORDER_REFLECT);
  }
  else {
    cv::warpPerspective(image,result,H,image.size(),  cv::INTER_LINEAR | cv::WARP_INVERSE_MAP,cv::BORDER_CONSTANT);
  }


}

void stabaliseImage(cv::Mat & image, cv::Mat &result, double roll, double pitch, double yaw)
{

  result.create(image.rows,image.cols,image.type());

  double stabalisedHorizonHeight = 0;
  double translation_v = -1* tan(pitch) * fy * scale;// - image.rows/2 + stabalisedHorizonHeight; // + v_offset;
  double translation_h = -1* tan(yaw)   * fx * scale; // + v_offset;

  cv::Mat rot_mat;
  double skew_amount = 0 ;

  //Rotate image
  if (roll != 0) {
    rot_mat = cv::getRotationMatrix2D(cv::Point(image.cols/2, image.rows/2 ),roll * -180/3.14159 ,1);
    cv::warpAffine(image, result,rot_mat,result.size(),cv::INTER_LINEAR,cv::BORDER_REFLECT,cv::Scalar(0,0,0));
  }

  // Translate
  rot_mat = cv::getRotationMatrix2D(cv::Point(image.cols/2, image.rows/2),0,1);
  rot_mat.at<double>(0,2) = translation_h ;
  rot_mat.at<double>(1,2) = translation_v;
  rot_mat.at<double>(0,1) = skew_amount;
  cv::warpAffine(result, result,rot_mat,result.size(),cv::INTER_NEAREST,cv::BORDER_REFLECT,cv::Scalar(0,0,0));

  // Draw the horizon
  cv::line(result,cv::Point(0,stabalisedHorizonHeight), cv::Point(image.cols-1,stabalisedHorizonHeight),cv::Scalar(128,128,128), 1.5 );

}

std::vector<double> averageColourPatch(cv::Mat& image){

    std::vector<double> avgColour;
    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    for(int i=0;  i<image.channels(); i++){
        cv::Mat avg;
        cv::reduce(channels[i],avg,1, CV_REDUCE_AVG, CV_64F);
        cv::reduce(avg,avg,0, CV_REDUCE_AVG, CV_64F);
        double v = avg.at<double>(0,0);
//        double v = u;
        avgColour.push_back(v);
        //std::cout << "Channel " << i << " has average of " << v  << std::endl;

    }

    return avgColour;

    //    cv::reduce(channels[0](cv::Range(0,5),cv::Range(0,result.cols-1)),avgTop,1, CV_REDUCE_AVG);
//    cv::reduce(channels[0](cv::Range(result.rows/3,result.rows-1),cv::Range(0,result.cols-1)),avgBot,1, CV_REDUCE_AVG);
//    cv::reduce(avgTop,avgTop,0, CV_REDUCE_AVG);
//    cv::reduce(avgBot,avgBot,0, CV_REDUCE_AVG);
//    int t = avgTop.at<uchar>(0,0);

}



std::vector<double> maxColourPatch(cv::Mat& image){

    std::vector<double> avgColour;
    std::vector<cv::Mat> channels;
    cv::split(image, channels);

    for(int i=0;  i<image.channels(); i++){
        cv::Mat avg;
        cv::reduce(channels[i],avg,1, CV_REDUCE_MAX);
        cv::reduce(avg,avg,0, CV_REDUCE_AVG);
        int u = avg.at<uchar>(0,0);
        double v = u;
        avgColour.push_back(v);
        //std::cout << "Channel " << i << " has average of " << v  << std::endl;

    }
    return avgColour;


}


int line_ransac(std::vector<cv::Point2i>& points, unsigned int min_inliers, unsigned int iterations, double tolerance,cv::Vec4f& result) {
    unsigned int max_inliers = 0;
    srand(time(NULL));


    for (unsigned int iter = 0; iter < iterations; iter++) {
        // Get a random sample...
        std::vector<cv::Point2i> sample;
        if (points.size() < 4) break;
        ///PRINT_VAL(points.size());
        for (unsigned int i = 0; i < 2; i++) sample.push_back(points[rand()%points.size()]);

        // Iteratively fit a line (untill you get enouogh points)
        cv::Vec4f temp_line;
        unsigned int inliers = 0;
        for (unsigned int i = 0; i < 10; i++) {
            // Fit a line to the data...
            cv::fitLine(cv::Mat(sample), temp_line, CV_DIST_L2, 0, 0.01, 0.01);

            // Re-estimate the samples
            inliers = 0;
            sample.clear();
            for (unsigned int j = 0; j < points.size(); j++) {
                double x = points[j].x,
                       y = points[j].y;
                double dist = fabs(temp_line[0]*(y-temp_line[3]) - temp_line[1]*(x-temp_line[2]));
                    if (dist <= tolerance) {
                        inliers++;
                        sample.push_back(points[j]);
                    }
                }
                if (sample.size() < 2) break;
//            if (inliers > min_inliers)  ;
        }

        double max_slope = 1;
        // Check the inliers
        if (inliers > max_inliers && inliers > min_inliers && fabs(temp_line[1]) < max_slope ) {
            max_inliers = inliers;
            result = temp_line;
            //std::cout << "New max inliers: " << max_inliers << std::endl;

//            break;
        }

    }
    return (max_inliers);
}



bool findHorizon(cv::Mat &image, double & roll_measured, double& pitch_measured){

    double h_top    = 20*scale; //pitchToRowNum(image, pitch_estimate - std::max(3 * sqrt(pitch_var), 0.1 ));
    double h_bot    = 180*scale;  //pitchToRowNum(image, pitch_estimate + std::max(3 * sqrt(pitch_var), 0.1 ));
    double h_centre = 90*scale; //pitchToRowNum(image, pitch_estimate);
    double left  = 300 * scale;
    double right = 300 * scale; // was 400

    // Limit the region to the top half of the image
    h_bot = std::min((double)image.rows/2,(double)h_bot);
    h_top = std::max(0.0, h_top);

    cv::Mat result = image.clone();


    cv::Rect roi(0+left,h_top ,image.cols-1-right,fabs(h_bot - h_top));
    try{
        result = result(roi);
    } catch (cv::Exception e) {

    }

    // Split into channels
    std::vector<cv::Mat> channels;
    cv::Mat b,g,r;
    cv::split(result, channels);
    channels[0].convertTo(b,CV_32F);
    channels[1].convertTo(g,CV_32F);
    channels[2].convertTo(r,CV_32F);

    // Define the top and bottom regions of interest
    double fraction = 4;
    cv::Rect top_roi(0,0 ,result.cols-1,result.rows/fraction);
    cv::Rect bot_roi(0,(fraction -1)*result.rows/fraction ,result.cols-1,result.rows/fraction -1);
    //cv::rectangle(result,top_roi, cv::Scalar(0,0,0));
    //cv::rectangle(result,bot_roi, cv::Scalar(0,0,0));

    cv::Mat top_patch = result(top_roi);
    cv::Mat bot_patch = result(bot_roi);

    std::vector<double> avgColourTop = averageColourPatch(top_patch);
    std::vector<double> avgColourBot = maxColourPatch(bot_patch);

    int HORIZON_THOROGOOD = 0;
    int HORIZON_ENGLISH  = 1;
    int HORIZON_BLUE    =  2;


    int method = HORIZON_THOROGOOD;
    cv::Mat thresholdedImg;
    if (method == HORIZON_ENGLISH )
    {
      // Calulate then normal vactor fo the plane that separates the two averages
      std::vector<double> n ;
      n.push_back(avgColourTop[0] - avgColourBot[0]);
      n.push_back(avgColourTop[1] - avgColourBot[1]);
      n.push_back(avgColourTop[2] - avgColourBot[2]);

      // Caluclate the point on the plant that separates the two medians
      std::vector<double> x0 ;
      x0.push_back( 0.5 *  (avgColourTop[0] + avgColourBot[0]));
      x0.push_back( 0.5 *  (avgColourTop[1] + avgColourBot[1]));
      x0.push_back( 0.5 *  (avgColourTop[2] + avgColourBot[2]));

      // Take the dot product of n and x0
      double nx0 = n[0] * x0[0] + n[1] * x0[1] + n[2] * x0[2];

      //std::cout << "avgColTop = " << avgColourTop.at(0) << "," <<avgColourTop.at(1) << "," <<avgColourTop.at(2) << "," << std::endl;
      //std::cout << "avgColBot = " << avgColourBot.at(0) << "," <<avgColourBot.at(1) << "," <<avgColourBot.at(2) << "," << std::endl;
      //std::cout << "n  = " << n[0] << "," <<n[1] << "," <<n[2] << "," << std::endl;
      //std::cout << "nx0  = " << nx0 << std::endl;

      cv::Mat combined;
      combined = n[0]*b + n[1]*g + n[2]*r  - nx0;
      cv::compare(thresholdedImg , 0, combined  , cv::CMP_GT);
      cv::imshow("combined", combined);

    }
    else if (method == HORIZON_BLUE)
    {
      // Threshold blue channel
      double t =  (0.5 * avgColourTop[0] + 0.5 * avgColourBot[0] ) ;
    //    std::cout << "threshold is: " << t<< std::endl;
      cv::compare(channels[0], t ,thresholdedImg, cv::CMP_LT);
      cv::imshow("blue", thresholdedImg);
    }
    else if (method == HORIZON_THOROGOOD)
    {
      //    // Thorowgood Version
      //        cv::Mat Thurrowgood = 0.2989 * r + 0.5870 * g + 0.1140 * b;

      cv::Mat Thurrowgood = -0.16 * r + 0.363*g + 1.43 * b - 82.3;
      cv::Mat top_patch_th = Thurrowgood(top_roi);
      cv::Mat bot_patch_th = Thurrowgood(bot_roi);
      bot_patch = bot_patch(cv::Range(0,bot_patch.rows-1),cv::Range(bot_patch.cols*0.9, bot_patch.cols*0.95));
      std::vector<double> avgColourTopTh = averageColourPatch(top_patch_th);
      std::vector<double> avgColourBotTh = averageColourPatch(bot_patch_th);
      double t_th =  (0.5 * avgColourTopTh[0] + 0.5 * avgColourBotTh[0] ) ;
      cv::compare(Thurrowgood, t_th ,thresholdedImg, cv::CMP_LT);
//      imshow("Thurrowgood ", Thurrowgood /256.0  );
//      imshow("ThurrowgoodThresh", thresholdedImg   );

    }
    else {
      ROS_ERROR_STREAM("Unknown horizon detection method: " << method);
      exit(EXIT_FAILURE);
    }


    // Find countours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresholdedImg, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE,cv::Point() ); //CV_CHAIN_APPROX_NONE or CV_CHAIN_APPROX_SIMPLE



    if(contours.size() <1 ) return false;

    // Find large contours
    std::vector<cv::Point> points_contour;
    std::vector<std::vector<cv::Point> > points_contours;
    for (std::size_t i = 0; i<contours.size(); i++){
        int contourSize = cv::arcLength(contours[i],false);
        //cv::Rect boundRect = cv::boundingRect(cv::Mat(contours[i])) ;
        if( (contourSize > 1) ){ // was 200
            points_contour.insert(points_contour.end(), contours[i].begin() , contours[i].end() );
            points_contours.push_back(contours[i]);
        }
    }

    // Remove points that are near the edges of the image.
    std::vector<cv::Point> points;
    for (int i = 0; i < points_contour.size() ; i++) {
        if (         ( points_contour[i].y > 10)
                &&  ( points_contour[i].y < result.rows -5)
                &&  ( points_contour[i].x > 10)
                &&  ( points_contour[i].x < result.cols -5)    ) {
            points.push_back(points_contour[i]);
            cv::circle(result , points_contour[i], 1, cv::Scalar(128,100,0),1 );
        }
    }
    points_contour = points;


    //Fit a straight line to this
    cv::Vec4f line;
    // cv::fitLine(cv::Mat(contours[largestContour]), line, CV_DIST_L1, 0.0, 0.01, 0.01);
    // int line_ransac(points, min_inliers, iterations, double tolerance,cv::Vec4f&
    int ransac_inliers = line_ransac(points_contour, 50 , 100, 2, line);

    // Now find two extreme points on the line to draw line
    double lefty = (-1*line[2]*line[1]/line[0] + line[3]);
    double righty = ((result.size[1]-line[2])*line[1]/line[0])+line[3];
    cv::Point p1(0,lefty);
    cv::Point p2(result.size[1],righty);
    cv::line(result,p1, p2,cv::Scalar(0,255,0),2);

    // Return the roll and pitch values
    roll_measured  = atan2(lefty-righty,result.cols);
    pitch_measured = atan2( (lefty + righty)/2 +h_top - image.rows/2 , fy * scale );

    cv::imshow("horizon region", result);

    cv::waitKey(10);

    return true;

}



void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& cam_info_msg)
{
//  camera_model.fromCameraInfo(cam_info_msg);
//  ROS_INFO("info callback");
}

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImuConstPtr& imu_msg)
{
//  ROS_INFO("Got a match between packets");

//  double fx, fy;
//  try {
//    fx = camera_model.fx();
//    fy = camera_model.fy();
//  } catch(...) {
//    ROS_INFO("No cameara info message recieved. Ignoring image");
//    return;
//  }


  tf::Quaternion q(imu_msg->orientation.x,imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
  tf::Matrix3x3 m(q);

  double roll , pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Check for missing frames
  static uint image_seq = image_msg->header.seq;
  if (image_msg->header.seq - 1 != image_seq ) {
    ROS_WARN("Skipped a frame");
  }
  image_seq = image_msg->header.seq;

  static double init_roll  = roll;
  static double init_pitch = pitch;
  static double init_yaw   = yaw;

  double delta_t = (image_msg->header.stamp - imu_msg->header.stamp).toSec();
  roll =  roll               + imu_msg->angular_velocity.x*delta_t;
  pitch = pitch - init_pitch + imu_msg->angular_velocity.y*delta_t ;

    if(fix_yaw)
    {
        yaw = -1*(yaw - init_yaw + imu_msg->angular_velocity.z*delta_t);
    }
    else
    {
      yaw = 0;
    }

//    bool filter_yaw = true;
    if(filter_yaw)
    {
      static double filtered_yaw ;
      static double prev_yaw = yaw;

      double alpha = 0.01;
      double delta_yaw = fmod( yaw - prev_yaw ,M_PI);
      if (delta_yaw > M_PI/2)
      {
        delta_yaw -= M_PI;
      }
      else if (delta_yaw < -M_PI/2)
      {
        delta_yaw += M_PI;
      }
      filtered_yaw = (1-alpha) * (filtered_yaw +  delta_yaw) + alpha * 0;
      filtered_yaw = fmod(filtered_yaw,M_PI);
      prev_yaw = yaw;
      yaw = filtered_yaw;
    }



  //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
//    cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);

    if (output_average_frame)
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    } else {
      cv_ptr = cv_bridge::toCvCopy(image_msg,      sensor_msgs::image_encodings::BGR8);
    }

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("crop_row_tracking::main.cpp::cv_bridge exception: %s", e.what());
    return;
  }

  //Undistort
  undistortImage(cv_ptr->image,cv_ptr->image);


  if (scale!= 1.0)
  {
    cv::resize(cv_ptr->image, cv_ptr->image ,cv::Size(),scale,scale);
  }

  bool enhance_contrast = false;
  if(enhance_contrast)
  {
    cv::Mat ycrcb;
    cv::cvtColor(cv_ptr->image,ycrcb,CV_BGR2HSV);
    std::vector<cv::Mat> channels;
    cv::split(ycrcb,channels);
    //  cv::equalizeHist(channels[0], channels[0]);
    //  cv::equalizeHist(channels[1], channels[1]);?
    cv::equalizeHist(channels[2], channels[2]);

    cv::merge(channels,ycrcb);
    cv::cvtColor(ycrcb,cv_ptr->image,CV_HSV2BGR);

    cv_ptr->image.convertTo(cv_ptr->image,-1,0.7,35);

  }

  if(!chop){
    int border = 150*scale;
    cv::copyMakeBorder(cv_ptr->image, cv_ptr->image, border,border,border,border, cv::BORDER_CONSTANT );
  }



  // Translate the image for pitch
  if(homography)
  {
    stabaliseImageHomography(cv_ptr->image, cv_ptr->image, roll, pitch  ,yaw);
  }
  else
  {
      stabaliseImage(cv_ptr->image, cv_ptr->image, roll, pitch  ,yaw);
  }


  if(chop)
  {
    int chop_top   = 20*scale;
    int chop_bot   = 180*scale;
    int chop_sides = 100*scale;
    cv::Rect roi(chop_sides,chop_top ,cv_ptr->image.cols-1-chop_sides*2,cv_ptr->image.rows-1-chop_top-chop_bot);
    cv_ptr->image = cv_ptr->image(roi);
  }

  if (find_horizon ){
    double roll_measured;
    double pitch_measured;
    findHorizon(cv_ptr->image, roll_measured, pitch_measured);
    static double init_roll_measured = roll_measured;
    static double init_pitch_measured = pitch_measured;
    ROS_INFO_ONCE(" roll, pitch, roll_error, pitch_error, roll_rate, pitch_rate ");
    ROS_INFO_ONCE("data= [");
    double roll_error = roll_measured - init_roll_measured;
    double pitch_error = pitch_measured - init_pitch_measured;
    std::cout << roll <<  ", " << pitch + init_pitch << ", " <<  roll_error << ", " << pitch_error<< ", " << imu_msg->angular_velocity.x << "," << imu_msg->angular_velocity.y << ";" << std::endl;

  }


  if (output_average_frame)
  {
    averageFrames(cv_ptr->image, cv_ptr->image);
  }


  cv::imshow("Image",cv_ptr->image);
  cv::waitKey(10);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "crop_row_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  cv::namedWindow("Image");

  std::string image_topic, imu_topic, camera_info_topic;
  bool compressed;


  private_nh.param("image", image_topic, std::string("/ptgrey_stereo/left/image_raw"));
  private_nh.param("compressed",compressed, false);
  private_nh.param("imu_topic", imu_topic, std::string("/vectornav/imu"));
  private_nh.param("camera_info_topic", camera_info_topic, std::string("/ptgrey_stereo/left/camera_info"));
  private_nh.param("scale",scale,1.0);
  private_nh.param("average",output_average_frame,true );
  private_nh.param("fix_yaw",fix_yaw,true);
  private_nh.param("scale_f",scale_f, 1.0);
  private_nh.param("find_horizon", find_horizon, false);
  private_nh.param("homography",homography,true);
  private_nh.param("chop",chop, true);
  private_nh.param("filter_yaw",filter_yaw, true);


  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter image_sub_;
  if (compressed)
  {
    image_sub_.subscribe(it, image_topic, 1, image_transport::TransportHints("compressed"));
  }
  else
  {
    image_sub_.subscribe(it, image_topic, 1);
  }
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_(private_nh, imu_topic, 1);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub_, imu_sub_);
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::Subscriber cam_info_sub = private_nh.subscribe(camera_info_topic,10,cameraInfoCallback);

  ROS_INFO("Initialised");

    ros::spin();

    return 0;
}




