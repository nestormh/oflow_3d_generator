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
#include <pcl/point_types.h>

#include <grull_elas_ros/ElasFrameData.h>

#include <elas/elas.h>

#include <chrono>

using namespace std;

#define INIT_CLOCK(start) auto start = std::chrono::high_resolution_clock::now();
#define END_CLOCK(time, start) float time = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now()-start).count();

namespace oflow_3d_generator {
    
class OFlow3dGenerator
{
public:
    OFlow3dGenerator(const std::string& transport);
    
protected:
    // Type definitions
    typedef image_transport::SubscriberFilter Subscriber;
    typedef message_filters::Subscriber<sensor_msgs::CameraInfo> InfoSubscriber;
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    
    // Callbacks
    void process(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& d_image_msg,
                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg);
    
    // Method functions
    void findPairsOFlow(const cv::Mat & img1, const cv::Mat & img2, 
                        vector<cv::Point2f> & outPoints1, vector<cv::Point2f> & outPoints2);
    void compute3DVectors(const vector<cv::Point2f> & origPoints, 
                            const vector<cv::Point2f> & destPoints, const cv::Mat & img, 
                            const cv::Mat & origDispImg, const cv::Mat & destDispImg,
                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & outputVectors);
    void get3DfromDisp(const cv::Mat & dispImg, 
                        const cv::Point2d point2D, cv::Point3d & point3D);
    
    // Properties
    deque<cv::Mat> m_leftImages;
    deque<cv::Mat> m_dispImages;
    image_geometry::StereoCameraModel m_model;
    Elas::parameters m_param;
    
    Subscriber m_left_sub, m_disp_sub;
    InfoSubscriber m_left_info_sub, m_right_info_sub;
    
    boost::shared_ptr<ExactSync> m_exact_sync;
    boost::shared_ptr<ApproximateSync> m_approximate_sync;
    boost::shared_ptr<Elas> m_elas;
    
    ros::Publisher m_flowVectorsPub;
};

}

#endif // OFLOW_3D_GENERATOR_H
