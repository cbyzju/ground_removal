#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

bool zsort(const pcl::PointXYZ a, const pcl::PointXYZ b)
{
    return (a.z < b.z);
}
void medianFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    std::sort(cloud->points.begin(), cloud->points.end(), zsort);
    int newsize = cloud->points.size() /2;
    cloud->points.resize(newsize);
}
ros::Publisher pub_plane, pub_object, pub_filtered_point;
//pcl::visualization::CloudViewer viewer("Cloud Viewer");

void removalCallback(const sensor_msgs::PointCloud2ConstPtr lidar_msg)
{
    double lidar_time = lidar_msg->header.stamp.toSec();

    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*lidar_msg, *laserCloudIn);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    
    bool down_sample = true;
    pcl::console::TicToc tt;
    tt.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point(new pcl::PointCloud<pcl::PointXYZ>);
    if(down_sample)
    {
        for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
        {
            pcl::PointXYZ point;
            point.x = laserCloudIn->points[i].x;
            point.y = laserCloudIn->points[i].y;
            point.z = laserCloudIn->points[i].z;
            if(point.x > -15 &&  point.x < 3 && 
               point.y >-15 && point.y<15 && point.z < -1)
            filtered_point->points.push_back(point);
        }
         //pcl::VoxelGrid<pcl::PointXYZ> sor;
         //sor.setInputCloud (laserCloudIn);
         //sor.setLeafSize (1.5, 1.5, 1.5);
         //sor.filter (*filtered_point);
    }
    else
         filtered_point = laserCloudIn;

    cout<<laserCloudIn->points.size()<<" points before downsample, "
        <<filtered_point->points.size()<<" points after downsample, ";
    bool median_filter = true;
    if(median_filter)
    {
        medianFilter(filtered_point);
    }
    cout<<filtered_point->points.size()<<" points after median filter"<<endl;
        
    pcl::SACSegmentation<pcl::PointXYZ> seg;// Create the segmentation object
    seg.setOptimizeCoefficients (true);// Optional
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);
    seg.setInputCloud (filtered_point);
    seg.segment (*inliers, *coefficients);

    cout<<"plane segementation cost "<< tt.toc()<<" ms, inliers = "<< inliers->indices.size ()<<endl;

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " " 
                                        << coefficients->values[3] << std::endl;


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points(new pcl::PointCloud<pcl::PointXYZRGB>);
  
    
    for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
    {
         
        pcl::PointXYZRGB point;
        point.x = laserCloudIn->points[i].x;
        point.y = laserCloudIn->points[i].y;
        point.z = laserCloudIn->points[i].z;

        float diff = abs(point.x*coefficients->values[0] + point.y*coefficients->values[1]
                        + point.z*coefficients->values[2] + coefficients->values[3]);

        if(diff > 0.3)
        {
                point.r = 0;
                point.g = 255;
                point.b = 0;
                object_points->points.push_back(point);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_points(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        pcl::PointXYZRGB point;
        point.x = filtered_point->points[inliers->indices[i]].x;
        point.y = filtered_point->points[inliers->indices[i]].y;
        point.z = filtered_point->points[inliers->indices[i]].z;
        point.r = 255;
        point.g = 255;
        point.b = 0;
        plane_points->points.push_back(point);
    }

    sensor_msgs::PointCloud2 plane;
    pcl::toROSMsg(*plane_points, plane);
    plane.header.stamp=ros::Time().fromSec(lidar_time);
    plane.header.frame_id = "pandar";
    pub_plane.publish(plane);

    sensor_msgs::PointCloud2 object;
    pcl::toROSMsg(*object_points, object);
    object.header.stamp=ros::Time().fromSec(lidar_time);
    object.header.frame_id = "pandar";
    pub_object.publish(object);
    
    sensor_msgs::PointCloud2 remained_point;
    pcl::toROSMsg(*filtered_point, remained_point);
    remained_point.header.stamp=ros::Time().fromSec(lidar_time);
    remained_point.header.frame_id = "pandar";
    pub_filtered_point.publish(remained_point);
    //viewer.showCloud(inlier_points);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ground_removal");
    ros::NodeHandle nh;
    std::string TOPIC = "pandar_points";

    ros::Subscriber point_cloud = nh.subscribe<sensor_msgs::PointCloud2>(TOPIC, 1, removalCallback);
    pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane_point", 1);
    pub_object = nh.advertise<sensor_msgs::PointCloud2>("object_point", 1);
    pub_filtered_point = nh.advertise<sensor_msgs::PointCloud2>("filtered_point", 1);
    ros::spin();
    return 0;
}