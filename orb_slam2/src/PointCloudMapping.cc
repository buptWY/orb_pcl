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
    this->resolution = resolution_;
    voxel.setLeafSize(resolution, resolution, resolution);

    globalMap = boost::make_shared<PointCloud>();
    OptimizedGlobalMap = boost::make_shared<PointCloud>();
    viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
}

void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    unique_lock <mutex> lck(keyframeMutex);
    keyframes.push_back(kf);
    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    keyFrameUpdated.notify_one();
}

void PointCloudMapping::shutdown()
{
    cout<<endl<<"--PointCloud Viewer ShutDown!"<<endl;
    {
        unique_lock <mutex> lck(shutDownMutex);
        shutDownFlag = true;
        unique_lock <mutex> lck2(keyframeMutex);
        keyFrameUpdated.notify_one();
    }
    viewerThread->detach();
}

void PointCloudMapping::viewer()
{
    while (1)
    {
        {
            unique_lock <mutex> lck_shutdown(shutDownMutex);
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock <mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
            keyFrameUpdated.wait(lck_keyframeUpdated);
        }

        //keyframe is updated
        size_t N = 0;
        {
            unique_lock <mutex> lck(keyframeMutex);
            N = keyframes.size();
        }

        for (size_t i = lastKeyframeSize; i < N; i++)
        {
            PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
            *globalMap += *p;
            p->clear();
        }

        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud(globalMap);
        voxel.filter(*tmp);
        globalMap->swap(*tmp);
        lastKeyframeSize = N;

        if(N%20==0)
        {
            cout<<"point cloud saved, point size="<<globalMap->points.size()<<endl;
            pcl::io::savePCDFileBinary ( "PointCloud.pcd", *globalMap );

            octomap::ColorOcTree tree(0.05);

            for (auto p:(*globalMap).points)
                tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);

            for (auto p:(*globalMap).points)
                tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );
            tree.updateInnerOccupancy();
            tree.write("Octomap.ot");

            boost::this_thread::sleep(boost::posix_time::microseconds (5000));
        }
    }
}

pcl::PointCloud<PointCloudMapping::PointT>::Ptr PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth)
{
    PointCloud::Ptr tmp(new PointCloud());
    for (int i = 0; i < depth.rows; i += 3)
    {
        for (int j = 0; j < depth.cols; j += 3)
        {
            float d = depth.ptr<float>(i)[j];
            if (d < 0.01 || d > 10.0 || isnan(d))
                continue;

            PointT p;

            p.z = d;
            p.x = (j - kf->cx) * p.z / kf->fx;
            p.y = (i - kf->cy) * p.z / kf->fy;

            p.r = color.ptr<uchar>(i)[j*3];
            p.g = color.ptr<uchar>(i)[j*3+1];
            p.b = color.ptr<uchar>(i)[j*3+2];

            tmp->points.push_back(p);
        }
    }

    std::vector<int> mapping;
    pcl::removeNaNFromPointCloud(*tmp, *tmp, mapping);

    Eigen::Isometry3d T = Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud);

    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    return cloud;
}


void PointCloudMapping::SavePointCloudMap(const string &filename)
{
    for(size_t i=0;i<keyframes.size();i++)                               // save the optimized pointcloud
    {
        if(i%10==0)
            cout<<"keyframe "<<i<<" ..."<<endl;
        PointCloud::Ptr tp = generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] );
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( tp );
        voxel.filter( *tmp );
        *OptimizedGlobalMap += *tmp;
    }
    pcl::io::savePCDFileBinary ( "/home/wangyang/exp_res/PointCloud.pcd", *OptimizedGlobalMap );
    cout<<endl<<"Save point cloud file successfully!"<<endl;
}

void PointCloudMapping::SaveOctoMap(const string &filename)
{
    octomap::ColorOcTree tree(0.05);
    for (auto p:(*OptimizedGlobalMap).points)
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);

    for (auto p:(*OptimizedGlobalMap).points)
        tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );

    tree.updateInnerOccupancy();
    tree.write("/home/wangyang/exp_res/OctoMap.ot");
    cout << endl << "Converting point cloud into color octomap done." << endl;
}

