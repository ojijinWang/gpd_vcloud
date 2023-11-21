#ifndef VCLOUD_GENERATOR_H
#define VCLOUD_GENERATOR_H
// *** Standard
#include <stdio.h>
#include <iostream>
// *** PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// *** OpenCV
#include <opencv2/opencv.hpp>

#include "pointcloud_downsize.h"

class vCloud_generator {

public:

	double vCloud_resolution_;
	double vCloud_thickness_;

	int pc_width_;
	int pc_height_;
	double pc_range_max_;
	double pc_range_min_;
	

	vCloud_generator(double resolution, double thickness, double range_max, double range_min);

	int generate_vCloud(const pcl::PointCloud<pcl::PointXYZ>& pc, pcl::PointCloud<pcl::PointXYZ>& vCloud_out);
	void get_points_type(const pcl::PointCloud<pcl::PointXYZ>& pc, 
		pcl::PointCloud<pcl::PointXYZ>& pc_surface_out,
		pcl::PointCloud<pcl::PointXYZ>& pc_left_out, 
		pcl::PointCloud<pcl::PointXYZ>& pc_right_out, 
		pcl::PointCloud<pcl::PointXYZ>& pc_up_out, 
		pcl::PointCloud<pcl::PointXYZ>& pc_down_out,
		pcl::PointCloud<pcl::PointXYZ>& pc_left_up_out, 
		pcl::PointCloud<pcl::PointXYZ>& pc_left_down_out, 
		pcl::PointCloud<pcl::PointXYZ>& pc_right_up_out, 
		pcl::PointCloud<pcl::PointXYZ>& pc_right_down_out);

	void get_edge_circle_points_tf(const pcl::PointCloud<pcl::PointXYZ>& pc_edge_left, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_edge_right, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_edge_up, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_edge_down, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_left, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_right, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_up, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_down);
	void get_corner_sphere_points_tf(const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_up, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_down, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_up, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_down, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_up, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_down, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_up, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_down);

	void get_surface_line_points(const pcl::PointCloud<pcl::PointXYZ>& pc_surface,
		pcl::PointCloud<pcl::PointXYZ>& pc_surface_line);

	void get_edge_circle_points(const pcl::PointCloud<pcl::PointXYZ>& pc_edge_left, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_edge_right, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_edge_up, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_edge_down, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_left, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_right, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_up, 
		pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_down);

	void get_corner_sphere_points(const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_up, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_down, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_up, 
		const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_down, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_up, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_down, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_up, 
		pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_down);

	void organization(const pcl::PointCloud<pcl::PointXYZ>& pc_in, pcl::PointCloud<pcl::PointXYZ>& pc_out);
	tf::StampedTransform get_transform(Eigen::Vector3d origin, Eigen::Vector3d vz_axisZ, Eigen::Vector3d vx_OXZ_support);

};

vCloud_generator::vCloud_generator(double resolution, double thickness, double range_min, double range_max)
{
	vCloud_resolution_ = resolution;
	vCloud_thickness_ = thickness;
	pc_range_min_ = range_min;
	pc_range_max_ = range_max;
}

int vCloud_generator::generate_vCloud(const pcl::PointCloud<pcl::PointXYZ>& pc_in, pcl::PointCloud<pcl::PointXYZ>& vCloud_out)
{
	//check if the cloud is a organized point
	pcl::PointCloud<pcl::PointXYZ> pc = pc_in; //ROS_INFO("vCloud : insert cloud_size: [height,width,size]: [%d, %d, %d]", (int)pc.height, (int)pc.width, (int)pc.size());
	if( pc.height < 2){ 
		ROS_INFO("vCloud : point cloud isn't organized. organize it");
		organization(pc_in, pc);
		pc_width_ = (int)pc.width;
		pc_height_ = (int)pc.height;
		
	}else{
		pc_width_ = (int)pc.width;
		pc_height_ = (int)pc.height;
	}

	ros::Time time_st = ros::Time::now();

	// get the type of points 
	pcl::PointCloud<pcl::PointXYZ> pc_surface;
	pcl::PointCloud<pcl::PointXYZ> pc_edge_left, pc_edge_right, pc_edge_up, pc_edge_down;
	pcl::PointCloud<pcl::PointXYZ> pc_corner_left_up, pc_corner_left_down, pc_corner_right_up, pc_corner_right_down;
	get_points_type(pc, pc_surface, pc_edge_left, pc_edge_right, pc_edge_up, pc_edge_down, pc_corner_left_up, pc_corner_left_down, pc_corner_right_up, pc_corner_right_down);

	// get the short line points around the surface points
	pcl::PointCloud<pcl::PointXYZ> pc_surface_line;
	get_surface_line_points(pc_surface, pc_surface_line);
	// get the edge circle points around the edge points
	pcl::PointCloud<pcl::PointXYZ> pc_edge_circle_left, pc_edge_circle_right, pc_edge_circle_up, pc_edge_circle_down;
	get_edge_circle_points_tf(pc_edge_left, pc_edge_right, pc_edge_up, pc_edge_down, pc_edge_circle_left, pc_edge_circle_right, pc_edge_circle_up, pc_edge_circle_down);
	//get_edge_circle_points(pc_edge_left, pc_edge_right, pc_edge_up, pc_edge_down, pc_edge_circle_left, pc_edge_circle_right, pc_edge_circle_up, pc_edge_circle_down);
	// get the corner sphere points around the corner points
	pcl::PointCloud<pcl::PointXYZ> pc_sphere_left_up, pc_sphere_left_down, pc_sphere_right_up, pc_sphere_right_down;
	get_corner_sphere_points_tf(pc_corner_left_up, pc_corner_left_down, pc_corner_right_up, pc_corner_right_down, pc_sphere_left_up, pc_sphere_left_down, pc_sphere_right_up, pc_sphere_right_down);
	//get_corner_sphere_points(pc_corner_left_up, pc_corner_left_down, pc_corner_right_up, pc_corner_right_down, pc_sphere_left_up, pc_sphere_left_down, pc_sphere_right_up, pc_sphere_right_down);
	
	// merge points
	pcl::PointCloud<pcl::PointXYZ> pc_vCloud;
	pc_vCloud = pc_surface_line;
	pc_vCloud += pc_edge_circle_left;
	pc_vCloud += pc_edge_circle_right; 
 	pc_vCloud += pc_edge_circle_up; 
	pc_vCloud += pc_edge_circle_down;
	pc_vCloud += pc_sphere_left_up;
	pc_vCloud += pc_sphere_left_down; 
	pc_vCloud += pc_sphere_right_up; 
	pc_vCloud += pc_sphere_right_down; 

	ros::Time time_ed = ros::Time::now();
	ROS_INFO("vCloud [Time, NumCloud, NumVcloud] %.4f, %d, %d", (time_st - time_ed).toSec(), (int)pc.size(),  (int)pc_vCloud.size());

	//rang filter only aceppt the point in front of camera
	pcl::PassThrough<pcl::PointXYZ> pass_range;
	pass_range.setFilterFieldName("z");
	pass_range.setFilterLimits(pc_range_min_, pc_range_max_);
	pass_range.filter(pc);

	//point downsize
	pointcloud_downsize(pc_vCloud, pc_vCloud, vCloud_resolution_);
	
	vCloud_out = pc_vCloud;
	return vCloud_out.size();
}

void vCloud_generator::get_points_type(const pcl::PointCloud<pcl::PointXYZ>& pc, 
	pcl::PointCloud<pcl::PointXYZ>& pc_surface_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_left_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_right_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_up_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_down_out,
	pcl::PointCloud<pcl::PointXYZ>& pc_left_up_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_left_down_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_right_up_out, 
	pcl::PointCloud<pcl::PointXYZ>& pc_right_down_out)
{
	pc_surface_out.clear();
	pc_left_out.clear();
	pc_right_out.clear();
	pc_up_out.clear();
	pc_down_out.clear();
	pc_left_up_out.clear(); 
	pc_left_down_out.clear();
	pc_right_up_out.clear();
	pc_right_down_out.clear();
	std::vector<bool> index_list_left, index_list_right, index_list_up, index_list_down;
	index_list_left.resize(pc.size());
	index_list_right.resize(pc.size());
	index_list_up.resize(pc.size());
	index_list_down.resize(pc.size());
	for(int k=0; k<pc.size(); k++)
	{
		index_list_left[k] = false;
		index_list_right[k] = false;
		index_list_up[k] = false;
		index_list_down[k] = false;
	}
	for(int y=1; y<pc_height_-1; y++)
	{
		for (int x=1; x<pc_width_-1; x++)
		{
			int index = y*pc_width_ + x;
			pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc.begin() + index;
			Eigen::Vector3d point(it->x, it->y, it->z);
			Eigen::Vector3d vector = point;
			double range = vector.norm();
			if(range <= pc_range_min_)	//the point is too close to the sensor
			{ continue; }
			if(range > pc_range_max_)	//FIXME
			{ continue; }
			if(fabs(it->z) < 0.000001)	//skip the zesros points
			{ printf("*"); continue; }

			pcl::PointCloud<pcl::PointXYZ>::const_iterator it_up = it - 1;
			pcl::PointCloud<pcl::PointXYZ>::const_iterator it_dn = it + 1;

			bool is_left_num = !(fabs(it_up->z) < 0.00001);
			bool is_right_num = !(fabs(it_dn->z) < 0.00001);

			//the left edge
			if(!is_left_num && is_right_num)
			{ 
				//find the left detected point & it should be not be closer then this
				for(int i=0; i<100; i++)
				{
					if(x-i <= 1)
					{ break; }
					pcl::PointCloud<pcl::PointXYZ>::const_iterator it_curent = it_up - i;
					bool is_current_num = !( fabs(it_curent->z) < 0.00001);
					if(is_current_num)
					{
						Eigen::Vector3d point_it_current(it_curent->x, it_curent->y, it_curent->z);
						double range_it_current = point_it_current.norm();
						if( range_it_current > range && (range_it_current - range) > vCloud_thickness_ )
						{ index_list_left[index] = true; }
						break; 
					}
				}
				continue; 
			}
			//the right edge
			else if(is_left_num && !is_right_num)
			{ 
				//find the left detected point & it should be not be closer then this
				for(int i=0; i<100; i++)
				{
					if(x+i >= pc_width_-1)
					{ break; }
					pcl::PointCloud<pcl::PointXYZ>::const_iterator it_curent = it_dn + i;
					bool is_current_num = !(fabs(it_curent->z) < 0.00001);
					if(is_current_num)
					{
						Eigen::Vector3d point_it_current(it_curent->x, it_curent->y, it_curent->z);
						double range_it_current = point_it_current.norm();
						if( range_it_current > range && (range_it_current - range) > vCloud_thickness_ )
						{ index_list_right[index] = true; }
						break; 
					}
				}
				continue; 
			}
			else if(is_left_num && is_right_num) //check if the neared points has a range
			{
				Eigen::Vector3d point_left_up(it_up->x, it_up->y, it_up->z);
				Eigen::Vector3d point_right_dn(it_dn->x, it_dn->y, it_dn->z);
				double range_left = point_left_up.norm();
				double range_right = point_right_dn.norm();
				bool is_left_gap = ( range_left > range && (range_left - range) > vCloud_thickness_ );
				bool is_right_gap = ( range_right > range && (range_right - range) > vCloud_thickness_ );
				if(is_left_gap && !is_right_gap)
				{ index_list_left[index] = true; }
				if(!is_left_gap && is_right_gap)
				{ index_list_right[index] = true; }
			}
		}
	}
	for (int y=1; y<pc_height_-1; y++)
	{
		for (int x=1; x<pc_width_-1; x++)
		{
			int index = y*pc_width_ + x;
			pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc.begin() + index;
			Eigen::Vector3d point(it->x, it->y, it->z);
			Eigen::Vector3d vector = point;
			double range = vector.norm();
			if(range <= pc_range_min_)	//the point is too close to the sensor
			{ continue; }
			if(range > pc_range_max_)	//FIXME
			{ continue; }
			if(fabs(it->z) < 0.000001)	//skip the zesros points
			{ continue; }

			pcl::PointCloud<pcl::PointXYZ>::const_iterator it_up = it - pc_width_;
			pcl::PointCloud<pcl::PointXYZ>::const_iterator it_dn = it + pc_width_;

			bool is_up_num = !(fabs(it_up->z) < 0.00001 );
			bool is_dn_num = !(fabs(it_dn->z) < 0.00001 );
			//the left edge
			if(!is_up_num && is_dn_num)
			{ 
				//find the left detected point & it should be not be closer then this
				for(int i=0; i<100; i++)
				{
					if(y-i <= 1)
					{ break; }
					pcl::PointCloud<pcl::PointXYZ>::const_iterator it_curent = it_up - i * pc_width_;
					bool is_current_num = !( fabs(it_curent->z) < 0.00001);
					if(is_current_num)
					{
						Eigen::Vector3d point_it_current(it_curent->x, it_curent->y, it_curent->z);
						double range_it_current = point_it_current.norm();
						if( range_it_current > range && (range_it_current - range) > vCloud_thickness_ )
						{ index_list_up[index] = true;}
						break; 
					}
				}
				continue; 
			}
			else if(is_up_num && !is_dn_num)
			{ 
				//find the left detected point & it should be not be closer then this
				for(int i=0; i<100; i++)
				{
					if(y+i >= pc_height_-1)
					{ break; }
					pcl::PointCloud<pcl::PointXYZ>::const_iterator it_curent = it_dn + i * pc_width_;
					bool is_current_num = !( fabs(it_curent->z) < 0.00001);
					if(is_current_num)
					{
						Eigen::Vector3d point_it_current(it_curent->x, it_curent->y, it_curent->z);
						double range_it_current = point_it_current.norm();
						if( range_it_current > range && (range_it_current - range) > vCloud_thickness_ )
						{ index_list_down[index] = true;}
						break; 
					}
				}
				continue; 
			}
			else if(is_up_num && is_dn_num) //check if the neared points has a range
			{
				Eigen::Vector3d point_up_up(it_up->x, it_up->y, it_up->z);
				Eigen::Vector3d point_dn_dn(it_dn->x, it_dn->y, it_dn->z);
				double range_up = point_up_up.norm();
				double range_dn = point_dn_dn.norm();
				bool is_up_gap = ( range_up > range && (range_up - range) > vCloud_thickness_ );
				bool is_dn_gap = ( range_dn > range && (range_dn - range) > vCloud_thickness_ );
				if(is_up_gap && !is_dn_gap)
				{ index_list_up[index] = true; }
				if(!is_up_gap && is_dn_gap)
				{ index_list_down[index] = true; }
			}
		}
	}
	for(int k=0; k<pc.size(); k++)
	{
		pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc.begin() + k;
		pcl::PointXYZ point(it->x, it->y, it->z);

		//surface short line
		if(!index_list_left[k] && !index_list_right[k] && !index_list_up[k] && !index_list_down[k]){
			pc_surface_out.push_back(point);
		}
		//left half circle
		if(index_list_left[k] && !index_list_right[k] && !index_list_up[k] && !index_list_down[k]){
			pc_left_out.push_back(point);
		}
		//right half circle
		if(!index_list_left[k] && index_list_right[k] && !index_list_up[k] && !index_list_down[k]){
			pc_right_out.push_back(point);
		}
		//up half circle
		if(!index_list_left[k] && !index_list_right[k] && index_list_up[k] && !index_list_down[k]){
			pc_up_out.push_back(point);
		}
		//down half circle
		if(!index_list_left[k] && !index_list_right[k] && !index_list_up[k] && index_list_down[k]){
			pc_down_out.push_back(point);
		}
		//left right circle
		if(index_list_left[k] && index_list_right[k] && !index_list_up[k] && !index_list_down[k]){
			pc_left_out.push_back(point);pc_right_out.push_back(point);
		}
		//up down circle
		if(index_list_left[k] && index_list_right[k] && !index_list_up[k] && !index_list_down[k]){
			pc_up_out.push_back(point);pc_down_out.push_back(point);
		}

		//left up qsphere
		if(index_list_left[k] && index_list_up[k]){
			pc_left_up_out.push_back(point);
		}
		//left down qsphere
		if(index_list_left[k] && index_list_down[k]){
			pc_left_down_out.push_back(point);
		}
		//right up qsphere
		if(index_list_right[k] && index_list_up[k]){
			pc_right_up_out.push_back(point);
		}
		//right down qsphere
		if(index_list_right[k] && index_list_down[k]){
			pc_right_down_out.push_back(point);
		}
	}
}


void vCloud_generator::get_surface_line_points(const pcl::PointCloud<pcl::PointXYZ>& pc_surface,
	pcl::PointCloud<pcl::PointXYZ>& pc_surface_line)
{
	pc_surface_line.clear();
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_surface.begin(); it != pc_surface.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_resolution_; l>-vCloud_thickness_; l-=vCloud_resolution_)
		{
			Eigen::Vector3d point_on_line = vector_norm * l;
			Eigen::Vector3d point_surround = point + point_on_line;
			pcl::PointXYZ point_line(point_surround(0), point_surround(1), point_surround(2));
			pc_surface_line.push_back(point_line);
		}
	}
}

void vCloud_generator::get_edge_circle_points(
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_left, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_right, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_up, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_down, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_left, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_right, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_up, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_down)
{
	pc_edge_circle_left.clear();
	pc_edge_circle_right.clear();
	pc_edge_circle_up.clear();
	pc_edge_circle_down.clear();
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_left.begin(); it != pc_edge_left.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double x = -vCloud_thickness_; x < 0; x+=vCloud_resolution_)
			{
				if(fabs(x) < vCloud_resolution_ && l >= 0) continue;
				Eigen::Vector3d point_goLeft(x, 0, 0);
				Eigen::Vector3d point_circle = point_circle_st + point_goLeft;
				if(point_circle.norm() > vCloud_thickness_)
				{ continue; }
				Eigen::Vector3d point_surround = point + point_circle;
				pcl::PointXYZ point_left(point_surround(0), point_surround(1), point_surround(2));
				pc_edge_circle_left.push_back(point_left);
			}
		}
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_right.begin(); it != pc_edge_right.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double x = 0; x < vCloud_thickness_; x+=vCloud_resolution_)
			{
				if(fabs(x) < vCloud_resolution_ && l >= 0) continue;
				Eigen::Vector3d point_goLeft(x, 0, 0);
				Eigen::Vector3d point_circle = point_circle_st + point_goLeft;
				if(point_circle.norm() > vCloud_thickness_)
				{ continue; }
				Eigen::Vector3d point_surround = point + point_circle;
				pcl::PointXYZ point_right(point_surround(0), point_surround(1), point_surround(2));
				pc_edge_circle_right.push_back(point_right);
			}
		}
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_up.begin(); it != pc_edge_up.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double y = -vCloud_thickness_; y < 0; y+=vCloud_resolution_)
			{
				if(fabs(y) < vCloud_resolution_ && l >= 0) continue;
				Eigen::Vector3d point_goUp(0, y, 0);
				Eigen::Vector3d point_circle = point_circle_st + point_goUp;
				if(point_circle.norm() > vCloud_thickness_)
				{ continue; }
				Eigen::Vector3d point_surround = point + point_circle;
				pcl::PointXYZ point_up(point_surround(0), point_surround(1), point_surround(2));
				pc_edge_circle_up.push_back(point_up);
			}
		}
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_down.begin(); it != pc_edge_down.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double y = 0; y < vCloud_thickness_; y+=vCloud_resolution_)
			{
				if(fabs(y) < vCloud_resolution_ && l >= 0) continue;
				Eigen::Vector3d point_goUp(0, y, 0);
				Eigen::Vector3d point_circle = point_circle_st + point_goUp;
				if(point_circle.norm() > vCloud_thickness_)
				{ continue; }
				Eigen::Vector3d point_surround = point + point_circle;
				pcl::PointXYZ point_down(point_surround(0), point_surround(1), point_surround(2));
				pc_edge_circle_down.push_back(point_down);
			}
		}
  	}
}

void vCloud_generator::get_edge_circle_points_tf(
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_left, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_right, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_up, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_edge_down, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_left, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_right, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_up, 
	pcl::PointCloud<pcl::PointXYZ>& pc_edge_circle_down)
{
	pc_edge_circle_left.clear();
	pc_edge_circle_right.clear();
	pc_edge_circle_up.clear();
	pc_edge_circle_down.clear();
	
	//generate a half-circle patterns
	pcl::PointCloud<pcl::PointXYZ> pc_circle_pattern;
	Eigen::Vector3d vector_norm(0,0,1);
	for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
	{
		Eigen::Vector3d point_circle_st = vector_norm * l;
		for(double x=0; x < vCloud_thickness_; x+=vCloud_resolution_)
		{
			if(fabs(x) < vCloud_resolution_ && l >= 0) continue;
			Eigen::Vector3d point_goRight(x, 0, 0);
			Eigen::Vector3d point_circle = point_circle_st + point_goRight;
			if(point_circle.norm() > vCloud_thickness_)
			{ continue; }
			pcl::PointXYZ point_left(point_circle(0), point_circle(1), point_circle(2));
			pc_circle_pattern.push_back(point_left);
		}
	}

	pc_edge_circle_left += pc_circle_pattern;
	
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_left.begin(); it != pc_edge_left.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_circle;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(1,0,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_circle_pattern, pc_circle, trans);

		pc_edge_circle_left += pc_circle;
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_right.begin(); it != pc_edge_right.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_circle;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(-1,0,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_circle_pattern, pc_circle, trans);

		pc_edge_circle_right += pc_circle;
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_up.begin(); it != pc_edge_up.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_circle;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(0,1,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_circle_pattern, pc_circle, trans);

		pc_edge_circle_up += pc_circle;
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_edge_down.begin(); it != pc_edge_down.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_circle;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(0,-1,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_circle_pattern, pc_circle, trans);

		pc_edge_circle_down += pc_circle;
  	}
}

void vCloud_generator::get_corner_sphere_points(
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_up, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_down, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_up, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_down, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_up, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_down, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_up, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_down)
{
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_left_up.begin(); it != pc_corner_left_up.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double x = -vCloud_thickness_; x < 0; x+=vCloud_resolution_)
			{
				for(double y = -vCloud_thickness_; y < 0; y+=vCloud_resolution_)
				{
					Eigen::Vector3d point_goLeftUp(x, y, 0);
					Eigen::Vector3d point_circle = point_circle_st + point_goLeftUp;
					if(point_circle.norm() > vCloud_thickness_)
					{ continue; }
					Eigen::Vector3d point_surround = point + point_circle;
					pcl::PointXYZ point_left_up(point_surround(0), point_surround(1), point_surround(2));
					pc_sphere_left_up.push_back(point_left_up);
				}
			}
		}
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_left_down.begin(); it != pc_corner_left_down.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double x = -vCloud_thickness_; x < 0; x+=vCloud_resolution_)
			{
				for(double y = 0; y < vCloud_thickness_; y+=vCloud_resolution_)
				{
					Eigen::Vector3d point_goLeftDown(x, y, 0);
					Eigen::Vector3d point_circle = point_circle_st + point_goLeftDown;
					if(point_circle.norm() > vCloud_thickness_)
					{ continue; }
					Eigen::Vector3d point_surround = point + point_circle;
					pcl::PointXYZ point_left_down(point_surround(0), point_surround(1), point_surround(2));
					pc_sphere_left_down.push_back(point_left_down);
				}
			}
		}
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_right_up.begin(); it != pc_corner_right_up.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double x = 0; x < vCloud_thickness_; x+=vCloud_resolution_)
			{
				for(double y = -vCloud_thickness_; y < 0; y+=vCloud_resolution_)
				{
					Eigen::Vector3d point_goRightUp(x, y, 0);
					Eigen::Vector3d point_circle = point_circle_st + point_goRightUp;
					if(point_circle.norm() > vCloud_thickness_)
					{ continue; }
					Eigen::Vector3d point_surround = point + point_circle;
					pcl::PointXYZ point_right_up(point_surround(0), point_surround(1), point_surround(2));
					pc_sphere_right_up.push_back(point_right_up);
				}
			}
		}
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_right_down.begin(); it != pc_corner_right_down.end(); ++it)
  	{
		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d vector_norm = point.normalized();
		for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
		{
			Eigen::Vector3d point_circle_st = vector_norm * l;
			for(double x = 0; x < vCloud_thickness_; x+=vCloud_resolution_)
			{
				for(double y = 0; y < vCloud_thickness_; y+=vCloud_resolution_)
				{
					Eigen::Vector3d point_goRightDown(x, y, 0);
					Eigen::Vector3d point_circle = point_circle_st + point_goRightDown;
					if(point_circle.norm() > vCloud_thickness_)
					{ continue; }
					Eigen::Vector3d point_surround = point + point_circle;
					pcl::PointXYZ point_right_down(point_surround(0), point_surround(1), point_surround(2));
					pc_sphere_right_down.push_back(point_right_down);
				}
			}
		}
  	}
}

void vCloud_generator::get_corner_sphere_points_tf(
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_up, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_left_down, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_up, 
	const pcl::PointCloud<pcl::PointXYZ>& pc_corner_right_down, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_up, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_left_down, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_up, 
	pcl::PointCloud<pcl::PointXYZ>& pc_sphere_right_down)
{
	pc_sphere_left_up.clear();
	pc_sphere_left_down.clear();
	pc_sphere_right_up.clear();
	pc_sphere_right_down.clear();

	//generate a quart-sphere patterns
	pcl::PointCloud<pcl::PointXYZ> pc_qsphere_pattern;
	Eigen::Vector3d vector_norm(0,0,1);
	for(double l=-vCloud_thickness_; l<vCloud_thickness_; l+=vCloud_resolution_)
	{
		Eigen::Vector3d point_circle_st = vector_norm * l;
		for(double x = 0; x < vCloud_thickness_; x+=vCloud_resolution_)
		{
			for(double y = 0; y < vCloud_thickness_; y+=vCloud_resolution_)
			{
				if(fabs(x) < vCloud_resolution_ && fabs(y) < vCloud_resolution_ && l >= 0) continue;
				Eigen::Vector3d point_goRightDown(x, y, 0);
				Eigen::Vector3d point_circle = point_circle_st + point_goRightDown;
				if(point_circle.norm() > vCloud_thickness_)
				{ continue; }
				pcl::PointXYZ point_right_down(point_circle(0), point_circle(1), point_circle(2));
				pc_qsphere_pattern.push_back(point_right_down);
			}
		}
	}

	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_left_up.begin(); it != pc_corner_left_up.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_qsphere;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(0,1,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_qsphere_pattern, pc_qsphere, trans);

		pc_sphere_left_up += pc_qsphere;
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_left_down.begin(); it != pc_corner_left_down.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_qsphere;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(1,0,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_qsphere_pattern, pc_qsphere, trans);

		pc_sphere_left_down += pc_qsphere;
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_right_up.begin(); it != pc_corner_right_up.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_qsphere;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(-1,0,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_qsphere_pattern, pc_qsphere, trans);

		pc_sphere_right_up += pc_qsphere;
  	}
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pc_corner_right_down.begin(); it != pc_corner_right_down.end(); ++it)
  	{
		pcl::PointCloud<pcl::PointXYZ> pc_qsphere;

		Eigen::Vector3d point(it->x, it->y, it->z);
		Eigen::Vector3d origin = point;
		Eigen::Vector3d vector_z = point.normalized();
		Eigen::Vector3d vector_x_OXZ_support(0,-1,0);
		tf::StampedTransform transTf = get_transform(origin, vector_z, vector_x_OXZ_support);
		Eigen::Matrix4f trans;
		pcl_ros::transformAsMatrix(transTf, trans);
		pcl::transformPointCloud(pc_qsphere_pattern, pc_qsphere, trans);

		pc_sphere_right_down += pc_qsphere;
  	}
}

void vCloud_generator::organization(const pcl::PointCloud<pcl::PointXYZ>& cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
	//generte a organize point map
	int width = 400;
	int height = 300;

	//generte a camera_transform map
	//pcl::PointCloud<pcl::PointXYZRGB> cloud_camera;
	pcl::PointCloud<pcl::PointXYZ> cloud_camera;
	int cloud_size = cloud_in.size();
	cloud_camera.resize(cloud_size);
	double x_min = 100000;
	double y_min = 100000;
	double z_min = 100000;
	double x_max = -100000;
	double y_max = -100000;
	double z_max = -100000;
	for (int index = 0; index<cloud_size; index++)
	{
		//pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it     = cloud.begin()        + index;
		//pcl::PointCloud<pcl::PointXYZRGB>::iterator       it_c   = cloud_camera.begin() + index;
		pcl::PointCloud<pcl::PointXYZ>::const_iterator it     = cloud_in.begin()        + index;
		pcl::PointCloud<pcl::PointXYZ>::iterator       it_c   = cloud_camera.begin() + index;
		if(fabs(it->z) < 0.00001)	//incase of the zero value
		{ continue; }
		it_c->x = atan2(it->x, it->z); //it->x / it->z; //
		it_c->y = it->y / it->z;
		it_c->z = it->z;
		if(x_min > it_c->x) { x_min = it_c->x; }
		if(y_min > it_c->y) { y_min = it_c->y; }
		if(z_min > it_c->z) { z_min = it_c->z; }
		if(x_max < it_c->x) { x_max = it_c->x; }
		if(y_max < it_c->y) { y_max = it_c->y; }
		if(z_max < it_c->z) { z_max = it_c->z; }
	}

	double width_guess = (x_max - x_min) / vCloud_resolution_ * 0.75;
	double height_guess = (y_max - y_min) / vCloud_resolution_ * 0.75;
	width = (int)width_guess;
	height = (int)height_guess;
	pcl::PointCloud<pcl::PointXYZ> cloud_organize(width, height, pcl::PointXYZ(0,0,0));

	//get the focus of the camera (can save all camera points into organize point map)
	double focus_x = width / (x_max - x_min) * 0.999;
	double focus_y = height / (y_max - y_min) * 0.999;

	for (int index = 0; index<cloud_size; index++)
	{
		pcl::PointCloud<pcl::PointXYZ>::const_iterator it     = cloud_in.begin()        + index;
		pcl::PointCloud<pcl::PointXYZ>::iterator       it_c   = cloud_camera.begin() + index;
		if(fabs(it->z) < 0.00001)
		{ continue; }
		int x_index = (int)floor((it_c->x - x_min) * focus_x);
		int y_index = (int)floor((it_c->y - y_min) * focus_y);
		int z__     = (int)floor(it_c->z);
		//pcl::PointCloud<pcl::PointXYZRGB>::iterator it_o	= cloud_organize.begin() + (int)y_index*width + x_index;
		pcl::PointCloud<pcl::PointXYZ>::iterator it_o	= cloud_organize.begin() + (int)(y_index*width + x_index);
		if(fabs(it->z) < 0.00001)	//incase of the zero value
		{ continue; }
		if(fabs(it_o->z) > 0.00001)	//there is a value in the pixel, keep the small one
		{
			double range   = sqrt(it->x*it->x + it->y*it->y + it->z*it->z);
			double range_o = sqrt(it_o->x*it_o->x + it_o->y*it_o->y + it_o->z*it_o->z);
			if(range > range_o)
			{ continue; }
		}
		it_o->x = it->x;
		it_o->y = it->y;
		it_o->z = it->z;
		//it_o->r = y_index%255;
		//it_o->g = y_index%255;
		//it_o->b = y_index%255;
	}

	//generate a depth image
	cv::Mat image(height, width, CV_8UC1, cv::Scalar(0));
	for(int row = 0; row < height; row++)
	{
		uchar* data = image.ptr<uchar>(row);
		for(int col = 0; col < width; col++)
		{
			int index = col + width*row;
			pcl::PointCloud<pcl::PointXYZ>::const_iterator it     = cloud_organize.begin()       + index;
			data[col] = (unsigned char)(it->z * 50);
		}
	}
	//imshow("depth map", image);
	//cv::waitKey(1);

	cloud_out = cloud_organize;
}

tf::StampedTransform vCloud_generator::get_transform(Eigen::Vector3d origin, Eigen::Vector3d vz_axisZ, Eigen::Vector3d vx_OXZ_support)
{
	tf::Vector3 po = tf::Vector3( origin[0], origin[1], origin[2]);
	tf::Vector3 vz = tf::Vector3( vz_axisZ[0], vz_axisZ[1], vz_axisZ[2]);
	tf::Vector3 vx_support = tf::Vector3( vx_OXZ_support[0], vx_OXZ_support[1], vx_OXZ_support[2]);
	vz = vz.normalize();
	vx_support = vx_support.normalize();

	//in case the vx and vy are not vertical to each other
	//first vy = vz cross vx
	//then vx = vy cross vz
	//get OXZ surface vector vy, which is the vz of Axis Y 
	tf::Vector3 vy = vz.cross(vx_support);
	vy = vy.normalize();

	//get vx, which is the vy of Axis X
	tf::Vector3 vx = - vy.cross(vz);
	vx = vx.normalize();

	//calculate the transform
	tf::Matrix3x3 trans_matrix(vx.x(), vy.x(), vz.x(), vx.y(), vy.y(), vz.y(), vx.z(), vy.z(), vz.z());
	tf::Transform trans;
	trans.setBasis(trans_matrix);
	trans.setOrigin(po);

	tf::StampedTransform stamped_trans(trans, ros::Time::now(), "parent_frame", "child_frame");

	return stamped_trans;
}


#endif
