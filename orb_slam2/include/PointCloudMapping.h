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

#include <octomap/octomap.h>

#include <thread>
#include <mutex>

using namespace ORB_SLAM2;

class PointCloudMapping
{
public:
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloudMapping(double resolution_);

    //Once a keyframe is inserted, the point cloud map will be updated
    void insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    void Run();

    void RequestFinish();

    bool isFinished();

    vector<KeyFrame*> GetKeyFrames();
    vector<cv::Mat> GetColorImgs();
    vector<cv::Mat> GetDepthImgs();
    octomap::OcTree GetOctoMap();

protected:
    PointCloud::Ptr generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth);

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mutexOfPCL; // if true will stop the thread

    vector<KeyFrame*> keyframes;
    vector<cv::Mat> colorImgs;
    vector<cv::Mat> depthImgs;

    std::mutex keyframeMutex;
    uint16_t lastKeyframeSize=0;

    Eigen::Matrix4f mTransCam2Ground;
    const double PI = 3.1415926535897932;


    PointCloud::Ptr mpPointCloudMap;
    pcl::VoxelGrid<PointT> mVoxel;

    double mResolution=0.05;

    shared_ptr<octomap::OcTree> mpOctree;
};
#endif // POINTCLOUDMAPPING_H
