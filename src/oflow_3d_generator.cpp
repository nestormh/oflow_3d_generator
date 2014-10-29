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
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <tiff.h>

namespace oflow_3d_generator {
    
OFlow3dGenerator::OFlow3dGenerator(const std::string& transport)
{
    ros::NodeHandle nh;
    int queue_size;
    
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 10);
    
    // Topics
    std::string stereo_ns = nh.resolveName("stereo");
    std::string left_topic = ros::names::clean(stereo_ns + "/left/" + nh.resolveName("image"));
    std::string right_topic = ros::names::clean(stereo_ns + "/right/" + nh.resolveName("image"));
    std::string disp_topic = ros::names::clean(stereo_ns + "/disparity/" + nh.resolveName("image"));
    std::string left_info_topic = stereo_ns + "/left/camera_info";
    std::string right_info_topic = stereo_ns + "/right/camera_info";
    
    image_transport::ImageTransport it(nh);
    m_left_sub.subscribe(it, left_topic, 1, transport);
    m_right_sub.subscribe(it, right_topic, 1, transport);
    m_disp_sub.subscribe(it, disp_topic, 1, transport);
    m_left_info_sub.subscribe(nh, left_info_topic, 1);
    m_right_info_sub.subscribe(nh, right_info_topic, 1);
    
    m_flowVectorsPub = nh.advertise<sensor_msgs::PointCloud2> ("flow_vectors", 1);
    
    // Check the frame w.r.t. motion is computed.
    local_nh.param<string>("motion_frame_id", m_motionFrame, "map");
    
    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
        m_approximate_sync.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                    m_left_sub, m_right_sub, m_disp_sub, m_left_info_sub, m_right_info_sub) );
        m_approximate_sync->registerCallback(boost::bind(&OFlow3dGenerator::process, this, _1, _2, _3, _4, _5));
    }
    else
    {
        m_exact_sync.reset(new ExactSync(ExactPolicy(queue_size),
                                            m_left_sub, m_right_sub, m_disp_sub, m_left_info_sub, m_right_info_sub) );
        m_exact_sync->registerCallback(boost::bind(&OFlow3dGenerator::process, this, _1, _2, _3, _4, _5));
    }
    
    
    // Create the elas processing class
    m_param = Elas::parameters(Elas::MIDDLEBURY);
    m_param.match_texture = 1;
    m_param.postprocess_only_left = 1;
    m_param.ipol_gap_width = 2;
    
    m_elas.reset(new Elas(m_param));
}    

void OFlow3dGenerator::process(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& r_image_msg,  
                                const sensor_msgs::ImageConstPtr& d_image_msg, 
                                const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    cv_bridge::CvImageConstPtr leftImgPtr, rightImgPtr, dispImgPtr;
    leftImgPtr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    rightImgPtr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    dispImgPtr = cv_bridge::toCvShare(d_image_msg, sensor_msgs::image_encodings::MONO8);
    m_model.fromCameraInfo(*l_info_msg, *r_info_msg);
    
    tf::StampedTransform tfCamera2Motion;
    try {
        m_tfListener.lookupTransform(m_motionFrame, l_info_msg->header.frame_id, ros::Time(0), tfCamera2Motion);
        
        // Accumulate images and frames, and process them
        m_leftImages.push_back(leftImgPtr->image);
        m_rightImages.push_back(rightImgPtr->image);
        m_dispImages.push_back(dispImgPtr->image);
        m_camera2MotionTransformation.push_back(tfCamera2Motion);
        
        if (m_leftImages.size() > 2) m_leftImages.pop_front();
        if (m_rightImages.size() > 2) m_rightImages.pop_front();
        if (m_dispImages.size() > 2) m_dispImages.pop_front();
        
        if (m_camera2MotionTransformation.size() > 2) m_camera2MotionTransformation.pop_front();
        
        if (m_leftImages.size() == 2) {
            INIT_CLOCK(startCompute)
            
            vector<cv::Point2f> points1, points2;
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr outputVectors;
            
            m_deltaTime = (m_camera2MotionTransformation[1].stamp_ - m_camera2MotionTransformation[0].stamp_).toSec();
            
            vector < vector < cv::Point2f > > correspondences;
//             findPairsOFlow(m_leftImages[0], m_leftImages[1], points1, points2);
            if (! findMatches(m_leftImages[0], m_rightImages[0], 
                m_leftImages[1], m_rightImages[1], correspondences, 1)) {
                                
                return;
            }
            points1 = correspondences[LT0];
            points2 = correspondences[LT1];
            compute3DVectors(points1, points2, m_leftImages[1], correspondences, m_dispImages[0], m_dispImages[1], outputVectors);
            
            // Publish results
            sensor_msgs::PointCloud2 cloudMsg;
            pcl::toROSMsg (*outputVectors, cloudMsg);
            cloudMsg.header.frame_id = m_motionFrame; // l_info_msg->header.frame_id;
            cloudMsg.header.stamp = ros::Time::now();
            m_flowVectorsPub.publish(cloudMsg);
            
            END_CLOCK(totalCompute, startCompute)
            
            ROS_INFO("[%s] Total time: %f seconds", __FUNCTION__, totalCompute);
        }
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
}

bool OFlow3dGenerator::findMatches(const cv::Mat & imgLt0, const cv::Mat & imgRt0, 
                                   const cv::Mat & imgLt1, const cv::Mat & imgRt1, 
                                   vector < vector < cv::Point2f > > & finalCorrespondences,
                                   const double & cornerThresh)
{
    vector < vector < cv::Point2f > > initialPoints(5);
    
    findInitialPoints(imgLt0, initialPoints[0], cornerThresh);

    findPairCorrespondencesOFlow(imgLt0, imgRt0, initialPoints[LT0], initialPoints[RT0]);
    findPairCorrespondencesOFlow(imgRt0, imgRt1, initialPoints[RT0], initialPoints[RT1]);
    findPairCorrespondencesOFlow(imgRt1, imgLt1, initialPoints[RT1], initialPoints[LT1]);
    findPairCorrespondencesOFlow(imgLt1, imgLt0, initialPoints[LT1], initialPoints[LT0B]);
    
    cleanCorrespondences(initialPoints, finalCorrespondences);
    
    return true;
}

inline 
void OFlow3dGenerator::findInitialPoints(const cv::Mat& img, vector< cv::Point2f >& points, const double & cornerThresh)
{
    vector<cv::KeyPoint> keypoints;
    cv::FastFeatureDetector fastDetector(cornerThresh);
    fastDetector.detect(img, keypoints);
    
    if (keypoints.size() == 0)
        return;
    
    points = vector<cv::Point2f>(keypoints.size());
    {
        uint32_t idx = 0;
        for (vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end(); it++, idx++) {
            points[idx] = it->pt;
        }
    }    
}

inline 
void OFlow3dGenerator::findPairCorrespondencesOFlow(const cv::Mat& img1, const cv::Mat& img2, 
                                                    vector< cv::Point2f >& points1, vector< cv::Point2f >& points2)
{
    // Optical flow
    vector<uint8_t> status, statusB;
    vector<float_t> error, errorB;
    
    cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, error, cv::Size(3, 3), 9);
}


inline 
void OFlow3dGenerator::cleanCorrespondences(const vector< vector< cv::Point2f > >& initialCorrespondences, 
                                                    vector< vector< cv::Point2f > >& finalCorrespondences)
{
    finalCorrespondences.resize(4);
    for (uint32_t i = 0; i < 4; i++) {
        finalCorrespondences[i].reserve(initialCorrespondences[LT0].size());
    }

    for (uint32_t i = 0; i < initialCorrespondences[LT0].size(); i++) {
        const cv::Point2f & p0 = initialCorrespondences[LT0][i];
        const cv::Point2f & p1 = initialCorrespondences[RT0][i];
        const cv::Point2f & p2 = initialCorrespondences[RT1][i];
        const cv::Point2f & p3 = initialCorrespondences[LT1][i];
        const cv::Point2f & p0b = initialCorrespondences[LT0B][i];
    
        const double & dist = cv::norm(p0 - p0b); 
//         sqrt((p0.x - p0b.x) * (p0.x - p0b.x) + 
//                            (p0.y - p0b.y) * (p0.y - p0b.y));
        if ((fabs(p0.y - p1.y) < MAX_HORIZONTAL_DIST) && (fabs(p2.y - p3.y) < MAX_HORIZONTAL_DIST) && 
//             (cv::norm(p1 - p2) < MAX_FLOW_DIST) && (cv::norm(p3 - p0b) < MAX_FLOW_DIST) &&
            (cv::norm(p0 - p0b) < MAX_CICLE_DIST)) {
            
            finalCorrespondences[LT0].push_back(p0);
            finalCorrespondences[RT0].push_back(p1);
            finalCorrespondences[RT1].push_back(p2);
            finalCorrespondences[LT1].push_back(p3);
        }
    }
}

// inline void OFlow3dGenerator::findPairsOFlow(const cv::Mat & img1, const cv::Mat & img2, 
//                                              vector<cv::Point2f> & outPoints1, vector<cv::Point2f> & outPoints2) 
// {
//     
//     // We look for correspondences using Optical flow
//     // vector of keypoints
//     vector<cv::KeyPoint> keypoints1;
//     cv::FastFeatureDetector fastDetector(1);
//     fastDetector.detect(img1, keypoints1);
//     
//     if (keypoints1.size() == 0)
//         return;
//     
//     vector<cv::Point2f> points1(keypoints1.size()), points2, points1B;
//     {
//         uint32_t idx = 0;
//         for (vector<cv::KeyPoint>::iterator it = keypoints1.begin(); it != keypoints1.end(); it++, idx++) {
//             points1[idx] = it->pt;
//         }
//     }    
//     // Optical flow
//     vector<uint8_t> status, statusB;
//     vector<float_t> error, errorB;
//     
//     cv::calcOpticalFlowPyrLK(img1, img2, points1, points2, status, error, cv::Size(3, 3), 3);
//     cv::calcOpticalFlowPyrLK(img2, img1, points2, points1B, statusB, errorB, cv::Size(3, 3), 3);
//     
//     vector<cv::Point2f> pointsA(points1.size()), pointsB(points2.size());
//     {
//         uint32_t idx = 0;
//         for (uint32_t i = 0; i < points1.size(); i++) {
//             if ((status[i] == 1) && (statusB[i] == 1)) {
//                 if (cv::norm(points1[i] - points1B[i]) < 1.0) {
//                     pointsA[idx] = points1[i];
//                     pointsB[idx] = points2[i];
//                 }
//             }
//             idx++;
//         }
//         pointsA.resize(idx);
//         pointsB.resize(idx);
//     }
//     
//     outPoints1 = pointsA;
//     outPoints2 = pointsB;
// //     outPoints1 = points1;
// //     outPoints2 = points2;
//     
// }

inline void OFlow3dGenerator::compute3DVectors(const vector<cv::Point2f> & origPoints, 
                                               const vector<cv::Point2f> & destPoints, const cv::Mat & img, 
                                               const vector < vector < cv::Point2f > > & correspondences, 
                                               const cv::Mat & origDispImg, const cv::Mat & destDispImg,
                                               pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & outputVectors) 
{
    outputVectors.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    outputVectors->reserve(origPoints.size());

    for (uint32_t i = 0; i < correspondences[LT0].size(); i++) {
        const double & dispOrigin = fabs(correspondences[RT0][i].x - correspondences[LT0][i].x);
        const double & dispDest = fabs(correspondences[RT1][i].x - correspondences[LT1][i].x);
        
        pcl::PointXYZRGBNormal flow;
        cv::Point3d  origPoint3D, destPoint3D, motOrigPoint3D, motDestPoint3D;

        m_model.projectDisparityTo3d(correspondences[LT0][i], dispOrigin, origPoint3D);
        m_model.projectDisparityTo3d(correspondences[LT1][i], dispDest, destPoint3D);
        
    /*
        if (! get3DfromDisp(origDispImg, *itOrig, origPoint3D)) continue;
        if (! get3DfromDisp(destDispImg, *itDest, destPoint3D)) continue;
    */
        transformPoint(origPoint3D, m_camera2MotionTransformation[0], motOrigPoint3D);
        transformPoint(destPoint3D, m_camera2MotionTransformation[1], motDestPoint3D);
    
        flow.x = motDestPoint3D.x;
        flow.y = motDestPoint3D.y;
        flow.z = motDestPoint3D.z;
    
//         flow.normal_x = motDestPoint3D.x - motOrigPoint3D.x;
//         flow.normal_y = motDestPoint3D.y - motOrigPoint3D.y;
//         flow.normal_z = motDestPoint3D.z - motOrigPoint3D.z;
        flow.normal_x = (motDestPoint3D.x - motOrigPoint3D.x) / m_deltaTime;
        flow.normal_y = (motDestPoint3D.y - motOrigPoint3D.y) / m_deltaTime;
        flow.normal_z = (motDestPoint3D.z - motOrigPoint3D.z) / m_deltaTime;
        
        flow.b = img.at<cv::Vec3b>(correspondences[LT1][i].y, correspondences[LT1][i].x)[0];
        flow.g = img.at<cv::Vec3b>(correspondences[LT1][i].y, correspondences[LT1][i].x)[1];
        flow.r = img.at<cv::Vec3b>(correspondences[LT1][i].y, correspondences[LT1][i].x)[2];
    
        outputVectors->push_back(flow);
    }
}

inline bool OFlow3dGenerator::get3DfromDisp(const cv::Mat & dispImg,
                                            const cv::Point2d point2D, cv::Point3d & point3D)
{
    const float dMax = (float)(m_param.disp_max);
    const double value = dispImg.at<uint8_t>(point2D.y, point2D.x);
    
    if (value == 0.0)
        return false;
    
    m_model.projectDisparityTo3d(point2D, value, point3D);
    
    return true;
}

inline void OFlow3dGenerator::transformPoint(const cv::Point3d & inputPoint3D, const tf::StampedTransform & tfCamera2Motion, 
                                             cv::Point3d & outputPoint3D)
{
    tf::Vector3 tmpPoint = tfCamera2Motion * tf::Vector3(inputPoint3D.x, inputPoint3D.y, inputPoint3D.z);
    outputPoint3D.x = tmpPoint.getX();
    outputPoint3D.y = tmpPoint.getY();
    outputPoint3D.z = tmpPoint.getZ();
}


}