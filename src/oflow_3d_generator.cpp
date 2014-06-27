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

namespace oflow_3d_generator {
    
OFlow_3d_generator::OFlow_3d_generator(const std::string& transport)
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
    
    m_dbgInputImg = it.advertise("dbg/input_img", 1);
    m_dbgDepthImg = it.advertise("dbg/depth_img", 1);
    m_dbgOthers = it.advertise("dbg/others_img", 1);
    
    // Synchronize input topics. Optionally do approximate synchronization.
    bool approx;
    local_nh.param("approximate_sync", approx, false);
    if (approx)
    {
        m_approximate_sync.reset(new ApproximateSync(ApproximatePolicy(queue_size),
                                                    m_left_sub, m_disp_sub, m_left_info_sub, m_right_info_sub) );
        m_approximate_sync->registerCallback(boost::bind(&OFlow_3d_generator::process, this, _1, _2, _3, _4));
    }
    else
    {
        m_exact_sync.reset(new ExactSync(ExactPolicy(queue_size),
                                        m_left_sub, m_disp_sub, m_left_info_sub, m_right_info_sub) );
        m_exact_sync->registerCallback(boost::bind(&OFlow_3d_generator::process, this, _1, _2, _3, _4));
    }
    
    // Create the elas processing class
    Elas::parameters param(Elas::MIDDLEBURY);
    param.match_texture = 1;
    param.postprocess_only_left = 1;
    param.ipol_gap_width = 2;
    
    #ifdef DOWN_SAMPLE
    param.subsampling = true;
    #endif
    m_elas.reset(new Elas(param));
}    

void OFlow_3d_generator::process(const sensor_msgs::ImageConstPtr& l_image_msg, const sensor_msgs::ImageConstPtr& d_image_msg, 
                                 const sensor_msgs::CameraInfoConstPtr& l_info_msg, const sensor_msgs::CameraInfoConstPtr& r_info_msg)
{
    cout << "Processing..." << endl;
    cv_bridge::CvImageConstPtr leftImgPtr, dispImgPtr;
    leftImgPtr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    dispImgPtr = cv_bridge::toCvShare(d_image_msg, sensor_msgs::image_encodings::MONO8);
    
    // TODO: Accumulate frames and process them
    
    publishMat(m_dbgInputImg, leftImgPtr->image);
    publishMat(m_dbgDepthImg, dispImgPtr->image);
}

}

void oflow_3d_generator::OFlow_3d_generator::publishMat(image_transport::Publisher& pub, const cv::Mat& img)
{
    cv::Mat colorImg;
    if (img.channels() == 1) {
        cv::cvtColor(img, colorImg, CV_GRAY2BGR);
    } else {
        colorImg = img;
    }
    sensor_msgs::Image msgImg;
    cv_bridge::CvImage bridgeImg(msgImg.header, sensor_msgs::image_encodings::BGR8, colorImg);
    
    pub.publish(bridgeImg.toImageMsg());
}
