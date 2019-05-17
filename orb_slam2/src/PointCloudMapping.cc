//
// Created by wangyang on 19-4-23.
//

#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include "Converter.h"

#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <boost/make_shared.hpp>

#include "Eigen/Dense"
#include "Eigen/Geometry"


PointCloudMapping::PointCloudMapping(double resolution_)
{
    mResolution = resolution_;
    mVoxel.setLeafSize(mResolution, mResolution, mResolution);

    mpPointCloudMap = boost::make_shared<PointCloud> ();
    mpOctree = make_shared<octomap::OcTree>(resolution_);

    mTransCam2Ground.setIdentity();
    mTransCam2Ground(0,0) = 0.0;
    mTransCam2Ground(0,1) = 0.0;
    mTransCam2Ground(0,2) = 1.0;
    mTransCam2Ground(0,3) = 0.0;
    mTransCam2Ground(1,0) = -1.0;
    mTransCam2Ground(1,1) = 0.0;
    mTransCam2Ground(1,2) = 0.0;
    mTransCam2Ground(1,3) = 0.0;
    mTransCam2Ground(2,0) = 0.0;
    mTransCam2Ground(2,1) = -1.0;
    mTransCam2Ground(2,2) = 0.0;
    mTransCam2Ground(2,3) = 0.0;

    mbFinishRequested = false;
    mbFinished = false;
    mutexOfPCL = false;   //lock the thread
}

void PointCloudMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool PointCloudMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void PointCloudMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool PointCloudMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    unique_lock <mutex> lck(keyframeMutex);
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    mutexOfPCL=true;    //the flag set to start mapping
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    PointCloud::Ptr tmp(new PointCloud());
    for (int i = 0; i < depth.rows; i += 5)
    {
        for (int j = 0; j < depth.cols; j += 5)
        {
            float d = depth.ptr<float>(i)[j];
            if (d < 0.01 || d > 4.0 || isnan(d))
                continue;
            PointT p;
            p.z = d;
            p.x = (j - kf->cx) * p.z / kf->fx;
            p.y = (i - kf->cy) * p.z / kf->fy;

            //if (p.x < -4.0 || p.x>4.0)     //pointcloud filter, 5m is farest boundary
            //    continue;
            //if (p.y < -4.0 || p.y>4.0)
            //    continue;

            p.r = uchar(color.at<cv::Vec3b>(i,j)[0]);
            p.g = uchar(color.at<cv::Vec3b>(i,j)[1]);
            p.b = uchar(color.at<cv::Vec3b>(i,j)[2]);
            tmp->points.push_back(p);
        }
    }

    Eigen::Isometry3d T = Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud());

    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;
    return cloud;
}


void PointCloudMapping::Run()
{
    while (1)
    {
        if(mutexOfPCL)
        {
            size_t N = 0;
            {
                unique_lock <mutex> lck(keyframeMutex);
                N = keyframes.size();
            }
            for (size_t i = lastKeyframeSize; i < N; i++)
            {
                PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
                PointCloud::Ptr tmp(new PointCloud());
                pcl::transformPointCloud(*p, *tmp, mTransCam2Ground.matrix());
                p->swap(*tmp);
                tmp->clear();
                *mpPointCloudMap += *p;
                for (auto pts:p->points)
                    mpOctree->updateNode(octomap::point3d(pts.x, pts.y, pts.z), true );
                p->clear();
            }
            PointCloud::Ptr tmp(new PointCloud());
            mVoxel.setInputCloud(mpPointCloudMap);
            mVoxel.filter(*tmp);
            mpPointCloudMap->swap(*tmp);
            lastKeyframeSize = N;
            tmp->clear();
        }
        mutexOfPCL = false;
        if(CheckFinish())
            break;
        usleep(20);
    }
    pcl::io::savePCDFileBinary ( "/home/wangyang/result/IncrementalPointCloud.pcd", *mpPointCloudMap );
    SetFinish();
}



vector<KeyFrame*> PointCloudMapping::GetKeyFrames()
{
    unique_lock <mutex> lck(keyframeMutex);
    return keyframes;
}


vector<cv::Mat> PointCloudMapping::GetDepthImgs()
{
    unique_lock <mutex> lck(keyframeMutex);
    return depthImgs;
}

vector<cv::Mat> PointCloudMapping::GetColorImgs()
{
    unique_lock <mutex> lck(keyframeMutex);
    return colorImgs;
}

octomap::OcTree PointCloudMapping::GetOctoMap()
{
    unique_lock <mutex> lck(keyframeMutex);
    mpOctree->updateInnerOccupancy();
    return *mpOctree;
}