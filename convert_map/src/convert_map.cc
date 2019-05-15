//
// Created by wangyang on 19-5-8.
//
#include <iostream>
#include <vector>
#include <string>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <octomap/octomap.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <octomap_ros/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <nav_msgs/OccupancyGrid.h>

void OctomapToOccupancyGridMsg(const octomap::OcTree &tree, nav_msgs::OccupancyGrid & ogmap, const double minZ_, const double maxZ_);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "all_maps");
    std::string map_frame_id_param("map");
    //ros::Time current_frame_time_(ros::Time::now());

    std::string name_of_node = ros::this_node::getName();
    ros::NodeHandle node_handle;

    ros::Publisher PointCloudMap_publisher = node_handle.advertise<sensor_msgs::PointCloud2>(name_of_node+"/point_cloud",10);
    ros::Publisher OctoMap_publisher = node_handle.advertise<octomap_msgs::Octomap>(name_of_node+"/octomap",10);
    ros::Publisher OccupancyGripMap_publisher = node_handle.advertise<nav_msgs::OccupancyGrid>(name_of_node+"/occupancygridmap",10);

    pcl::PointCloud<pcl::PointXYZ> PointCloudMap;
    octomap::OcTree ocTreeMap(0.05);

    if(pcl::io::loadPCDFile("/home/wangyang/result/GlobalPointCloudMap.pcd", PointCloudMap)==-1)
    {
        std::cout<<"Could not read pcd file." <<std::endl;
        return -1;
    }
    for (auto p:PointCloudMap.points)
        ocTreeMap.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    ocTreeMap.updateInnerOccupancy();

    sensor_msgs::PointCloud2 pcmsg;
    pcl::toROSMsg (PointCloudMap, pcmsg);
    pcmsg.header.stamp = ros::Time::now();
    pcmsg.header.frame_id = map_frame_id_param;

    octomap_msgs::Octomap octo_msg;
    octomap_msgs::fullMapToMsg(ocTreeMap, octo_msg);
    octo_msg.header.stamp = ros::Time::now();
    octo_msg.header.frame_id = map_frame_id_param;

    nav_msgs::OccupancyGrid ogmsg;
    OctomapToOccupancyGridMsg(ocTreeMap, ogmsg, 0.5, 1.5);
    ogmsg.header.stamp = ros::Time::now();
    ogmsg.header.frame_id = map_frame_id_param;

    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        PointCloudMap_publisher.publish(pcmsg);
        OctoMap_publisher.publish(octo_msg);
        OccupancyGripMap_publisher.publish(ogmsg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

/*
void OctomapToOccupancyGridMsg(const octomap::OcTree &tree, nav_msgs::OccupancyGrid &ogmap)
{
    ogmap.info.resolution = tree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    tree.getMetricMin(minX, minY, minZ);
    tree.getMetricMax(maxX, maxY, maxZ);
    std::cout<<"Octree min:  "<<minX<<"  "<<minY<<"  "<<minZ<<std::endl;
    std::cout<<"Octree max:  "<<maxX<<"  "<<maxY<<"  "<<maxZ<<std::endl;

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    if (!tree.coordToKeyChecked(minPt, minKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }
    if (!tree.coordToKeyChecked(maxPt, maxKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    ogmap.info.width = maxKey[0] - minKey[0] + 1;
    ogmap.info.height = maxKey[1] - minKey[1] + 1;

    // might not exactly be min / max:
    octomap::point3d origin =  tree.keyToCoord(minKey, tree.getTreeDepth());

    /
     * Aligns base_link with origin of map frame, but is not correct in terms of real environment
     * (real map's origin is in the origin of camera's origin)
     * map.info.origin.position.x = origin.x() - octree.getResolution() * 0.5 - cam_base_translation_.at<float>(0);
     * map.info.origin.position.y = origin.y() - octree.getResolution() * 0.5 - cam_base_translation_.at<float>(1);
     *
     /

    ogmap.info.origin.position.x = origin.x() - tree.getResolution() * 0.5;
    ogmap.info.origin.position.y = origin.y() - tree.getResolution() * 0.5;

    ogmap.info.origin.orientation.x = 0.;
    ogmap.info.origin.orientation.y = 0.;
    ogmap.info.origin.orientation.z = 0.;
    ogmap.info.origin.orientation.w = 1.;

    // Allocate space to hold the data
    ogmap.data.resize(ogmap.info.width * ogmap.info.height, -1);

    // Matrix of map's size is inited with unknown (-1) value at each point
    for(std::vector<int8_t>::iterator it = ogmap.data.begin(); it != ogmap.data.end(); ++it)
        *it = -1;

    unsigned i, j;
    // iterate over all keys:
    for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
    {
        for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
        {
            for (curKey[2] = minKey[2]+0.5; curKey[2] <= maxKey[2]-1.4; ++curKey[2])
            {   //iterate over height
                octomap::OcTreeNode* node = tree.search(curKey);
                if (node)
                {
                    if(tree.isNodeOccupied(node))
                    {
                        ogmap.data[ogmap.info.width * j + i] = 100;
                        break;
                    }
                    else
                        ogmap.data[ogmap.info.width * j + i] = 0;
                }
            }
        }
    }
}
*/

void OctomapToOccupancyGridMsg(const octomap::OcTree &tree, nav_msgs::OccupancyGrid & ogmap, const double minZ_, const double maxZ_)
{
    ogmap.info.resolution = tree.getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    tree.getMetricMin(minX, minY, minZ);
    tree.getMetricMax(maxX, maxY, maxZ);
    std::cout<<"Octree min:  "<<minX<<"  "<<minY<<"  "<<minZ<<std::endl;
    std::cout<<"Octree max:  "<<maxX<<"  "<<maxY<<"  "<<maxZ<<std::endl;

    minZ = std::max(minZ_+minZ, minZ);
    maxZ = std::min(minZ-minZ_+maxZ_, maxZ);
    std::cout<<"threshold min:  "<<minZ<<std::endl;
    std::cout<<"threshold max:  "<<maxZ<<std::endl;

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;

    if (!tree.coordToKeyChecked(minPt, minKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
        return;
    }

    if (!tree.coordToKeyChecked(maxPt, maxKey))
    {
        ROS_ERROR("Could not create OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
        return;
    }

    ogmap.info.width = maxKey[0] - minKey[0] + 1;
    ogmap.info.height = maxKey[1] - minKey[1] + 1;

    // might not exactly be min / max:
    octomap::point3d origin =  tree.keyToCoord(minKey, tree.getTreeDepth());
    ogmap.info.origin.position.x = origin.x() - tree.getResolution() * 0.5;
    ogmap.info.origin.position.y = origin.y() - tree.getResolution() * 0.5;

    ogmap.info.origin.orientation.x = 0.;
    ogmap.info.origin.orientation.y = 0.;
    ogmap.info.origin.orientation.z = 0.;
    ogmap.info.origin.orientation.w = 1.;

    // Allocate space to hold the data
    ogmap.data.resize(ogmap.info.width * ogmap.info.height, -1);

    // Matrix of map's size is inited with unknown (-1) value at each point
    for(std::vector<int8_t>::iterator it = ogmap.data.begin(); it != ogmap.data.end(); ++it)
        *it = -1;

    unsigned i, j;
    // iterate over all keys:
    for (curKey[1] = minKey[1], j = 0; curKey[1] <= maxKey[1]; ++curKey[1], ++j)
    {
        for (curKey[0] = minKey[0], i = 0; curKey[0] <= maxKey[0]; ++curKey[0], ++i)
        {
            for (curKey[2] = minKey[2]; curKey[2] <= maxKey[2]; ++curKey[2])
            {   //iterate over height
                octomap::OcTreeNode* node = tree.search(curKey);
                if (node)
                {
                    if(tree.isNodeOccupied(node))
                    {
                        ogmap.data[ogmap.info.width * j + i] = 100;
                        break;
                    }
                    else
                        ogmap.data[ogmap.info.width * j + i] = 0;
                }
            }
        }
    }
}
