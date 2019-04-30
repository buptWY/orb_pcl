//
// Created by wangyang on 19-4-23.
//

#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include "System.h"
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <condition_variable>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution_);

    //Once a keyframe is inserted, the point cloud map will be updated
    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    void shutdown();
    void viewer();

    vector<KeyFrame*> GetKeyFrames();
    vector<cv::Mat> GetColorImgs();
    vector<cv::Mat> GetDepthImgs();



protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);
    PointCloud::Ptr globalMap;

    shared_ptr<thread> viewerThread;

    bool shutDownFlag = false;
    mutex shutDownMutex;

    condition_variable keyFrameUpdated;
    mutex keyFrameUpdateMutex;

    //data used to generate point clouds
    vector<KeyFrame*> keyframes;
    vector<cv::Mat> colorImgs;
    vector<cv::Mat> depthImgs;

    mutex keyframeMutex;
    uint16_t lastKeyframeSize=0;
    double resolution = 0.04;
    pcl::VoxelGrid<PointT> voxel;

};
#endif // POINTCLOUDMAPPING_H
