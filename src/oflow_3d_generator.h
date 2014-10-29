/*
 *  Copyright 2014 Néstor Morales Hernández <nestor@isaatc.ull.es>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef OFLOW_3D_GENERATOR_H
#define OFLOW_3D_GENERATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/subscriber_filter.h>

#include <image_geometry/stereo_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl/point_types.h>

#include <grull_elas_ros/ElasFrameData.h>

#include <elas/elas.h>

#include <chrono>

using namespace std;

#define INIT_CLOCK(start) auto start = std::chrono::high_resolution_clock::now();
#define END_CLOCK(time, start) float time = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now()-start).count();

#define MAX_HORIZONTAL_DIST 99999.0 //1.0
#define MAX_FLOW_DIST 99999.0 //20.0
#define MAX_CICLE_DIST 5.0 //1.0

namespace oflow_3d_generator {
    
class OFlow3dGenerator
{
public:
    OFlow3dGenerator(const std::string& transport);
    
protected:
    // Constants
    static const uint8_t LT0 = 0;
    static const uint8_t RT0 = 1;
    static const uint8_t RT1 = 2;
    static const uint8_t LT1 = 3;
    static const uint8_t LT0B = 4;
    
    // Type definitions
    typedef image_transport::SubscriberFilter Subscriber;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    
    // Callbacks
    void process(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg,  
                 const sensor_msgs::ImageConstPtr& d_image_msg, 
                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg);
    
    // Method functions
//     void findPairsOFlow(const cv::Mat & img1, const cv::Mat & img2, 
//                         vector<cv::Point2f> & outPoints1, vector<cv::Point2f> & outPoints2);
    static bool findMatches(const cv::Mat & imgLt0, const cv::Mat & imgRt0, 
                            const cv::Mat & imgLt1, const cv::Mat & imgRt1, 
                            vector < vector < cv::Point2f > > & finalCorrespondences,
                            const double & cornerThresh = 1);
    static void findInitialPoints(const cv::Mat & img, vector<cv::Point2f> & points, const double & cornerThresh);
    static void findPairCorrespondencesOFlow(const cv::Mat & img1, const cv::Mat & img2, 
                                             vector<cv::Point2f> & points1, vector<cv::Point2f> & points2);
    static void cleanCorrespondences(const vector < vector < cv::Point2f > > & initialCorrespondences,
                                     vector < vector < cv::Point2f > > & finalCorrespondences);
    void compute3DVectors(const vector<cv::Point2f> & origPoints, 
                            const vector<cv::Point2f> & destPoints, const cv::Mat & img, 
                            const vector < vector < cv::Point2f > > & correspondences, 
                            const cv::Mat & origDispImg, const cv::Mat & destDispImg,
                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & outputVectors);
    bool get3DfromDisp(const cv::Mat & dispImg, 
                        const cv::Point2d point2D, cv::Point3d & point3D);
    void transformPoint(const cv::Point3d & inputPoint3D, const tf::StampedTransform & tfCamera2Motion, 
                        cv::Point3d & outputPoint3D);
    
    // Properties
    deque<cv::Mat> m_leftImages;
    deque<cv::Mat> m_rightImages;
    deque<cv::Mat> m_dispImages;
    deque<tf::StampedTransform> m_camera2MotionTransformation;
    image_geometry::StereoCameraModel m_model;
    Elas::parameters m_param;
    string m_motionFrame;
    tf::TransformListener m_tfListener;
    
    double m_deltaTime;
    
    Subscriber m_left_sub, m_right_sub, m_disp_sub;
    InfoSubscriber m_left_info_sub, m_right_info_sub;
    
    boost::shared_ptr<ExactSync> m_exact_sync;
    boost::shared_ptr<ApproximateSync> m_approximate_sync;
    boost::shared_ptr<Elas> m_elas;
    
    ros::Publisher m_flowVectorsPub;
};

}

#endif // OFLOW_3D_GENERATOR_H
