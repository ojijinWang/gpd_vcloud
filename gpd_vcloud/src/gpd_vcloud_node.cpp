// *** Standard
#include <stdio.h>
#include <iostream>
// *** ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
// *** PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
// *** Custom
#include "vCloud_generator.h"
#include "grasp_detector_vcloud.h"
#include "grasp_visual_markers.h"
#include "pointcloud_downsize.h"
#include <gpg/cloud_camera.h>
#include <gpg/candidates_generator.h>

// *** param
std::string base_link_frame_id_;
// *** publihsers
ros::Publisher grasps_selected_rviz_pub;
ros::Publisher grasps_candidate_rviz_pub;
ros::Publisher grasps_candidate_vcloud_rviz_pub;
ros::Publisher grasps_pub;
ros::Publisher pc_out_pub;
ros::Publisher pc_vcloud_out_pub;
// *** tf_ listener
tf::TransformListener* listener_;

bool is_cloud = false;
sensor_msgs::PointCloud2 cloud_msg;
void chatter_pcIn_Callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in_msg)
{
	cloud_msg.header = cloud_in_msg->header;
	cloud_msg.height = cloud_in_msg->height;
	cloud_msg.width = cloud_in_msg->width;
	cloud_msg.fields = cloud_in_msg->fields;
	cloud_msg.is_bigendian = cloud_in_msg->is_bigendian;
	cloud_msg.point_step = cloud_in_msg->point_step;
	cloud_msg.row_step = cloud_in_msg->row_step;
	cloud_msg.data = cloud_in_msg->data;
	cloud_msg.is_dense = cloud_in_msg->is_dense;
	is_cloud = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gpd_vcloud_node");
	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_("~");

	if(!private_nh_.getParam("base_link_frame_id", base_link_frame_id_)) { ROS_ERROR("gpd_vcloud_node : cant find base_link_frame_id prama. end"); return 0;}

	// *** TF listener
	tf::TransformListener listener(ros::Duration(36000));
	listener_ = &listener;

	// *** publihsers to publish grasps
	grasps_selected_rviz_pub = nh_.advertise<visualization_msgs::MarkerArray>("selected_grasp", 1);
	grasps_candidate_rviz_pub = nh_.advertise<visualization_msgs::MarkerArray>("candidate_grasp", 1);
	grasps_candidate_vcloud_rviz_pub = nh_.advertise<visualization_msgs::MarkerArray>("candidate_grasp_filtered_by_vcloud", 1);
	grasps_pub = nh_.advertise<gpd_vcloud::GraspConfigList>("detected_grasps", 1);

  	// *** Publishers to send pointcloud
	pc_out_pub = nh_.advertise<sensor_msgs::PointCloud2>("cloud_used", 1);
	pc_vcloud_out_pub = nh_.advertise<sensor_msgs::PointCloud2>("vcloud", 1);

	// *** Subscriber to get in pointcloud
  	ros::Subscriber chatter_pcIn_sub = nh_.subscribe("cloud_in", 1, chatter_pcIn_Callback);

	// *** define the generator
	GraspDetector graspDetector(private_nh_);

	// *** main loop
	ros::Rate loop_rate(25);
	while(nh_.ok()){
		// *** wait point cloud
		while(!is_cloud){
			ROS_INFO("gpd_vcloud_node: waiting point cloud");
			ros::spinOnce();
			ros::Duration(0.5).sleep();
		}

		// *** generate vCloud
		pcl::PointCloud<pcl::PointXYZ> cloud;
		pcl::fromROSMsg (cloud_msg, cloud);
		vCloud_generator vCloud_g(0.005, 0.05, 0.3, 0.7);
		pcl::PointCloud<pcl::PointXYZ> vCloud;
		vCloud_g.generate_vCloud(cloud, vCloud);

		// *** downsize pointcloud
		ROS_WARN("gpd_vcloud_node: vcloud size: %d", (int)vCloud.size());
		pointcloud_downsize(cloud, cloud, 0.005);
		pointcloud_downsize(vCloud, vCloud, 0.005);
		ROS_WARN("gpd_vcloud_node: vcloud size: %d", (int)vCloud.size());

		// *** transfrom the point cloud to robot base
		// ** get transform of laser to base
		tf::StampedTransform sensorToWorldTf;
		try {
			listener_->lookupTransform(base_link_frame_id_, cloud_msg.header.frame_id, cloud_msg.header.stamp, sensorToWorldTf);
		} catch(tf::TransformException& ex){
			ROS_ERROR_STREAM( "gpd_vcloud_node : Transform error of sensor data: " << ex.what() << ", quitting callback");
			return false;
		}
		Eigen::Matrix4f sensorToWorld;
		pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
		// ** transform to robot base frame:
		pcl::transformPointCloud(cloud, cloud, sensorToWorld);
		pcl::transformPointCloud(vCloud,  vCloud,  sensorToWorld);

		// *** limitation of workspace of point cloud
		double x_min = -1;
		double x_max = 1;
		double y_min = -1;
		double y_max = 1;
		double z_min = -1;
		double z_max = 1;
		pcl::PassThrough<pcl::PointXYZ> pass_x;
		pass_x.setFilterFieldName("x");
		pass_x.setFilterLimits(x_min, x_max);
		pcl::PassThrough<pcl::PointXYZ> pass_y;
		pass_y.setFilterFieldName("y");
		pass_y.setFilterLimits(y_min, y_max);
		pcl::PassThrough<pcl::PointXYZ> pass_z;
		pass_z.setFilterFieldName("z");
		pass_z.setFilterLimits(z_min, z_max);
		pass_x.setInputCloud(cloud.makeShared());
		pass_x.filter(cloud);
		pass_y.setInputCloud(cloud.makeShared());
		pass_y.filter(cloud);
		pass_z.setInputCloud(cloud.makeShared());
		pass_z.filter(cloud);
		// *** limitation of workspace of point cloud
		double x_min_cloud = 100000;
		double y_min_cloud = 100000;
		double z_min_cloud = 100000;
		double x_max_cloud = -100000;
		double y_max_cloud = -100000;
		double z_max_cloud = -100000;
		for (int index = 0; index<cloud.size(); index++){
			pcl::PointCloud<pcl::PointXYZ>::const_iterator it     = cloud.begin()	+ index;
			if(fabs(it->z) < 0.1)	//incase of the zero value
			{ continue; }
			if(x_min_cloud > it->x) { x_min_cloud = it->x; }
			if(y_min_cloud > it->y) { y_min_cloud = it->y; }
			if(z_min_cloud > it->z) { z_min_cloud = it->z; }
			if(x_max_cloud < it->x) { x_max_cloud = it->x; }
			if(y_max_cloud < it->y) { y_max_cloud = it->y; }
			if(z_max_cloud < it->z) { z_max_cloud = it->z; }
		}
		ROS_INFO("vcloud workspace reize: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", x_min_cloud, x_max_cloud, y_min_cloud, y_max_cloud, z_min_cloud, z_max_cloud);
		double dxx = 0.05; //fabs(x_max- x_min);
		double dyy = 0.05; //fabs(y_max- y_min);
		double dzz = 0.05; //fabs(z_max- z_min);
		pcl::PassThrough<pcl::PointXYZ> pass_v_x;
		pass_v_x.setFilterFieldName("x");
		pass_v_x.setFilterLimits(x_min_cloud-dxx, x_max_cloud+dxx);
		pcl::PassThrough<pcl::PointXYZ> pass_v_y;
		pass_v_y.setFilterFieldName("y");
		pass_v_y.setFilterLimits(y_min_cloud-dyy, y_max_cloud+dyy);
		pcl::PassThrough<pcl::PointXYZ> pass_v_z;
		pass_v_z.setFilterFieldName("z");
		pass_v_z.setFilterLimits(z_min_cloud-dzz, z_max_cloud+dzz);
		pass_v_x.setInputCloud(vCloud.makeShared());
		pass_v_x.filter(vCloud);
		pass_v_y.setInputCloud(vCloud.makeShared());
		pass_v_y.filter(vCloud);
		pass_v_z.setInputCloud(vCloud.makeShared());
		pass_v_z.filter(vCloud);
		ROS_WARN("vcloud size: %d", (int)vCloud.size());

		// *** get pcMsg & publish the vcloud for debuging
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg<pcl::PointXYZ> (cloud, output);
		output.header.frame_id = base_link_frame_id_;
		pc_out_pub.publish(output);
		pcl::toROSMsg<pcl::PointXYZ> (vCloud, output);
		output.header.frame_id = base_link_frame_id_;
		pc_vcloud_out_pub.publish(output);

		// *** generate grasp pose
		std::vector<double> viewpoint{0, 0, 1};
		std::vector<Grasp> candidate_grasps_list_origin;	// the candidate for debuging
		std::vector<Grasp> candidate_grasps_list_vcloud_filter; // the candidate filtered by vloud for debuging
		std::vector<Grasp> selected_grasps_list;	// the selected candidate filtered by vloud and classifier
		std::vector<double> scores;
		graspDetector.detectGrasps(viewpoint, cloud, vCloud, selected_grasps_list, scores, candidate_grasps_list_origin, candidate_grasps_list_vcloud_filter);
		if(selected_grasps_list.size() <= 0)
		{ return false; }

		// *** nomarlize classfier score
		std::vector<double> classfier_score_list((int)selected_grasps_list.size());
		double max = 0;
		double min = 1;
		for(int i=0; i<classfier_score_list.size(); i++)
		{
			if(max < selected_grasps_list[i].getScore()) { max = selected_grasps_list[i].getScore(); }
			if(min > selected_grasps_list[i].getScore()) { min = selected_grasps_list[i].getScore(); }
		}
		for(int i=0; i<classfier_score_list.size(); i++)
		{ classfier_score_list[i] = (selected_grasps_list[i].getScore() - min)/(max - min); } //ROS_WARN("%.2f", classfier_score_list[i]);
		//nomarlized the visibility score
		std::vector<double> visibility_score_list((int)selected_grasps_list.size());
		max = 0; min = 1;
		for(int i=0; i<visibility_score_list.size(); i++)
		{
			if(max < scores[i]) { max = scores[i]; }
			if(min > scores[i]) { min = scores[i]; }
		}
		for(int i=0; i<visibility_score_list.size(); i++)
		{ visibility_score_list[i] = (scores[i] - min)/(max - min); }
		//tatal score
		std::vector<double> grasps_score_list((int)selected_grasps_list.size());
		for(int i=0; i<grasps_score_list.size(); i++)
		{ grasps_score_list[i] = 0.5 * classfier_score_list[i] + 0.5 * visibility_score_list[i]; }

		// *** set score into grasps
		for(int i=0; i<selected_grasps_list.size(); i++)
		{ selected_grasps_list[i].setScore(grasps_score_list[i]); }
		
		// *** Publish the selected grasps.
		gpd_vcloud::GraspConfigList selected_grasps_msg = createGraspListMsg(selected_grasps_list);
		grasps_pub.publish(selected_grasps_msg);
		ROS_INFO_STREAM("gpd_vcloud : Published " << selected_grasps_msg.grasps.size() << " highest-scoring grasps.");

		// *** visualize
		grasps_selected_rviz_pub.publish(VisualGraspMsg_deleteAll());
		grasps_selected_rviz_pub.publish(convertToVisualGraspMsg_vcloud(selected_grasps_list, 0.09, 0.06, 0.01, 0.02, base_link_frame_id_, grasps_score_list, 0,1,1,1));
		grasps_candidate_rviz_pub.publish(VisualGraspMsg_deleteAll());
		grasps_candidate_rviz_pub.publish(convertToVisualGraspMsg(candidate_grasps_list_origin, 0.09, 0.06, 0.01, 0.02, base_link_frame_id_, 0,0,1,1));
		grasps_candidate_vcloud_rviz_pub.publish(VisualGraspMsg_deleteAll());
		grasps_candidate_vcloud_rviz_pub.publish(convertToVisualGraspMsg(candidate_grasps_list_vcloud_filter, 0.09, 0.06, 0.01, 0.02, base_link_frame_id_, 0,1,1,1));
				
		break;
		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spinOnce();
	return 0;
}

