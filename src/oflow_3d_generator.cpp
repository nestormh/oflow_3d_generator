/*
    Copyright 2014 Néstor Morales Hernández <nestor@isaatc.ull.es>

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "oflow_3d_generator.h"

#include <ros/ros.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

namespace oflow_3d_generator {
    
OFlow3dGenerator::OFlow3dGenerator(const std::string& transport)
{
    ros::NodeHandle nh;
    int queue_size;
    
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 5);
    
    // Topics
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/disparity/" + nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";
    
    image_transport::ImageTransport it(nh);
    m_left_sub.subscribe(it, left_topic, 1, transport);
    m_disp_sub.subscribe(it, right_topic, 1, transport);
    m_left_info_sub.subscribe(nh, left_info_topic, 1);
    m_right_info_sub.subscribe(nh, right_info_topic, 1);
    
    m_flowVectorsPub = nh.advertise<sensor_msgs::PointCloud2> ("flow_vectors", 1);
    
    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
        m_approximate_sync.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                    m_left_sub, m_disp_sub, m_left_info_sub, m_right_info_sub) );
        m_approximate_sync->registerCallback(boost::bind(&OFlow3dGenerator::process, this, _1, _2, _3, _4));
    }
    else
    {
        m_exact_sync.reset(new ExactSync(ExactPolicy(queue_size),
                                        m_left_sub, m_disp_sub, m_left_info_sub, m_right_info_sub) );
        m_exact_sync->registerCallback(boost::bind(&OFlow3dGenerator::process, this, _1, _2, _3, _4));
    }
    
    // Create the elas processing class
    m_param = Elas::parameters(Elas::MIDDLEBURY);
    m_param.match_texture = 1;
    m_param.postprocess_only_left = 1;
    m_param.ipol_gap_width = 2;
    
    m_elas.reset(new Elas(m_param));
}    

void OFlow3dGenerator::process(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& d_image_msg, 
                                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    cv_bridge::CvImageConstPtr leftImgPtr, dispImgPtr;
    leftImgPtr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    dispImgPtr = cv_bridge::toCvShare(d_image_msg, sensor_msgs::image_encodings::MONO8);
    m_model.fromCameraInfo(*l_info_msg, *r_info_msg);
    
    // TODO: Accumulate frames and process them
    m_leftImages.push_back(leftImgPtr->image);
    m_dispImages.push_back(dispImgPtr->image);
    
    if (m_leftImages.size() > 2) m_leftImages.pop_front();
    if (m_dispImages.size() > 2) m_dispImages.pop_front();
    
    if (m_leftImages.size() == 2) {
        vector<cv::Point2f> points1, points2;
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputVectors;
        findPairsOFlow(m_leftImages[0], m_leftImages[1], points1, points2);
        compute3DVectors(points1, points2, m_leftImages[1], m_dispImages[0], m_dispImages[1], outputVectors);
        
        // Publish results
        sensor_msgs::PointCloud2 cloudMsg;
        pcl::toROSMsg (*outputVectors, cloudMsg);
        cloudMsg.header.frame_id = l_info_msg->header.frame_id;
        cloudMsg.header.stamp = ros::Time();
        m_flowVectorsPub.publish(cloudMsg);
    }
}

inline void OFlow3dGenerator::findPairsOFlow(const cv::Mat & img1, const cv::Mat & img2, 
                                             vector<cv::Point2f> & outPoints1, vector<cv::Point2f> & outPoints2) 
{
    
    // We look for correspondences using Optical flow
    // vector of keypoints
    vector<cv::KeyPoint> keypoints1;
    cv::FastFeatureDetector fastDetector(50);
    fastDetector.detect(img1, keypoints1);
    
    if (keypoints1.size() == 0)
        return;
    
    vector<cv::Point2f> points1(keypoints1.size()), points2, points1B;
    {
        uint32_t idx = 0;
        for (vector<cv::KeyPoint>::iterator it = keypoints1.begin(); it != keypoints1.end(); it++, idx++) {
            points1[idx] = it->pt;
        }
    }    
    // Optical flow
    vector<uint8_t> status, statusB;
    vector<float_t> error, errorB;
    
    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, error, cv::Size(3, 3), 3);
    cv::calcOpticalFlowPyrLK(img2, img1, points2, points1B, statusB, errorB, cv::Size(3, 3), 3);
    
    vector<cv::Point2f> pointsA(points1.size()), pointsB(points2.size());
    {
        uint32_t idx = 0;
        for (uint32_t i = 0; i < points1.size(); i++) {
            if ((status[i] == 1) && (statusB[i] == 1)) {
                if (cv::norm(points1[i] - points1B[i]) < 1.0) {
                    pointsA[idx] = points1[i];
                    pointsB[idx] = points2[i];
                }
            }
            idx++;
        }
        pointsA.resize(idx);
        pointsB.resize(idx);
    }
    
    outPoints1 = pointsA;
    outPoints2 = pointsB;
    
}

inline void OFlow3dGenerator::compute3DVectors(const vector<cv::Point2f> & origPoints, 
                                               const vector<cv::Point2f> & destPoints, const cv::Mat & img, 
                                               const cv::Mat & origDispImg, const cv::Mat & destDispImg,
                                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & outputVectors) 
{
    outputVectors.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    outputVectors->reserve(origPoints.size());

    for (vector<cv::Point2f>::const_iterator itOrig = origPoints.begin(), itDest = destPoints.begin();
            itOrig != origPoints.end(); itOrig++, itDest++) {
                
        pcl::PointXYZRGBNormal flow;
        cv::Point3d  origPoint3D, destPoint3D;
        get3DfromDisp(origDispImg, *itOrig, origPoint3D);
        get3DfromDisp(destDispImg, *itDest, destPoint3D);
    
        flow.x = destPoint3D.x;
        flow.y = destPoint3D.y;
        flow.z = destPoint3D.z;
    
        flow.normal_x = destPoint3D.x - origPoint3D.x;
        flow.normal_y = destPoint3D.y - origPoint3D.y;
        flow.normal_z = destPoint3D.z - origPoint3D.z;
        
        flow.b = img.at<cv::Vec3b>(itDest->y, itDest->x)[0];
        flow.g = img.at<cv::Vec3b>(itDest->y, itDest->x)[1];
        flow.r = img.at<cv::Vec3b>(itDest->y, itDest->x)[2];
    
        outputVectors->push_back(flow);
    }
}

inline void OFlow3dGenerator::get3DfromDisp(const cv::Mat & dispImg, 
                                            const cv::Point2d point2D, cv::Point3d & point3D)
{
    const float dMax = (float)(m_param.disp_max);
    const double value = dispImg.at<uint8_t>(point2D.y, point2D.x); //dMax * dispImg.at<uint8_t>(point2D.y, point2D.x) / 255;
    m_model.projectDisparityTo3d(point2D, value, point3D);
}

}