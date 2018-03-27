#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <iostream>
using namespace std;

ros::Publisher pub_plane_points, pub_roi_points, pub_noground_points;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

bool down_sample = true;
bool median_filter = true;

bool zsort(const pcl::PointXYZI a, const pcl::PointXYZI b)
{
    return (a.z < b.z);
}
void medianFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    std::sort(cloud->points.begin(), cloud->points.end(), zsort);
    int newsize = cloud->points.size() /2;
    cloud->points.resize(newsize);
}

void removalCallback(const sensor_msgs::PointCloud2ConstPtr lidar_msg)
{
    double lidar_time = lidar_msg->header.stamp.toSec();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
    //pcl::fromROSMsg(*lidar_msg, *laserCloudIn);
    sensor_msgs::PointCloud2 lidar = *lidar_msg;
    sensor_msgs::PointCloud2Iterator<float> iter_x(lidar, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(lidar, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(lidar, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_i(lidar, "intensity"); //uint8_t for rslidar, float for velodyne
    for (int i = 0; iter_z != iter_z.end(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
        pcl::PointXYZI point;
        point.x = (*iter_x);
        point.y = (*iter_y);
        point.z = (*iter_z);     
        point.intensity = unsigned(*iter_i);
        laserCloudIn->points.push_back(point);
    }

    pcl::console::TicToc tt;
    tt.tic();
    pcl::PointCloud<pcl::PointXYZI>::Ptr roi_points(new pcl::PointCloud<pcl::PointXYZI>);
    if(down_sample)
    {
        for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
        {
            pcl::PointXYZI point;
            point.x = laserCloudIn->points[i].x;
            point.y = laserCloudIn->points[i].y;
            point.z = laserCloudIn->points[i].z;
            point.intensity = laserCloudIn->points[i].intensity;
            if(point.x > -15 &&  point.x < 3 &&  point.y >-15 && point.y<15 && point.z < -1)
                    roi_points->points.push_back(point);
        }
         //pcl::VoxelGrid<pcl::PointXYZ> sor;
         //sor.setInputCloud (laserCloudIn);
         //sor.setLeafSize (1.5, 1.5, 1.5);
         //sor.filter (*filtered_point);
    }
    else
         roi_points = laserCloudIn;

    cout<<laserCloudIn->points.size()<<" points before downsample, "
        <<roi_points->points.size()<<" points after downsample, ";
    
    if(median_filter)
    {
        medianFilter(roi_points);
    }
    cout<<roi_points->points.size()<<" points after median filter"<<endl;
        
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZI> seg;// Create the segmentation object
    seg.setOptimizeCoefficients (true);// Optional
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (roi_points);
    seg.segment (*inliers, *coefficients);

    cout<<"plane segementation cost "<< tt.toc()<<" ms, inliers = "<< inliers->indices.size ()<<endl;

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr noground_points(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
    {
        pcl::PointXYZI point;
        point.x = laserCloudIn->points[i].x;
        point.y = laserCloudIn->points[i].y;
        point.z = laserCloudIn->points[i].z;

        float diff = abs(point.x*coefficients->values[0] + point.y*coefficients->values[1]
                        + point.z*coefficients->values[2] + coefficients->values[3]);

        if(diff > 0.3)
        {
            point.intensity = laserCloudIn->points[i].intensity;
            noground_points->points.push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZI>);
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        pcl::PointXYZI point;
        point.x = roi_points->points[inliers->indices[i]].x;
        point.y = roi_points->points[inliers->indices[i]].y;
        point.z = roi_points->points[inliers->indices[i]].z;
        point.intensity = roi_points->points[inliers->indices[i]].intensity;
        plane_points->points.push_back(point);
    }

    sensor_msgs::PointCloud2 plane_points_msgs;
    pcl::toROSMsg(*plane_points, plane_points_msgs);
    plane_points_msgs.header.stamp=ros::Time().fromSec(lidar_time);
    plane_points_msgs.header.frame_id = "pandar";
    pub_plane_points.publish(plane_points_msgs);

    sensor_msgs::PointCloud2 object_points_msgs;
    pcl::toROSMsg(*noground_points, object_points_msgs);
    object_points_msgs.header.stamp=ros::Time().fromSec(lidar_time);
    object_points_msgs.header.frame_id = "pandar";
    pub_noground_points.publish(object_points_msgs);
    
    sensor_msgs::PointCloud2 roi_points_msg;
    pcl::toROSMsg(*roi_points, roi_points_msg);
    roi_points_msg.header.stamp=ros::Time().fromSec(lidar_time);
    roi_points_msg.header.frame_id = "pandar";
    pub_roi_points.publish(roi_points_msg);
    //viewer.showCloud(inlier_points);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ground_removal");
    ros::NodeHandle nh;
    std::string TOPIC = "pandar_points";

    ros::Subscriber point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(TOPIC, 1, removalCallback);
    pub_plane_points = nh.advertise<sensor_msgs::PointCloud2>("plane_points", 1);
    pub_noground_points = nh.advertise<sensor_msgs::PointCloud2>("noground_points", 1);
    pub_roi_points = nh.advertise<sensor_msgs::PointCloud2>("roi_points", 1);
    ros::spin();
    return 0;
}