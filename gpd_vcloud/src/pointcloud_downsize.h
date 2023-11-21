#ifndef POINTCLOUD_DOWNSIZE_H
#define POINTCLOUD_DOWNSIZE_H

// *** Standard
#include <stdio.h>
#include <iostream>
// *** PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

void pointcloud_downsize(pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out, double resolution)
{
	//point downsize
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr (new pcl::PointCloud<pcl::PointXYZ>); *pc_ptr = cloud_in;
	pcl::VoxelGrid<pcl::PointXYZ> sor_pc;
	sor_pc.setInputCloud (pc_ptr);
	sor_pc.setLeafSize (resolution, resolution, resolution);
	pcl::PointCloud<pcl::PointXYZ> cloud_inner;
	sor_pc.filter(cloud_inner);
	cloud_out = cloud_inner;
}

#endif
