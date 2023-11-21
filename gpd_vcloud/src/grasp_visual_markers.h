#ifndef GRASP_VISUAL_MARKERS_H
#define GRASP_VISUAL_MARKERS_H

// *** Standard
#include <stdio.h>
#include <iostream>
// *** ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// *** Custom
#include "gpd_vcloud/CloudIndexed.h"
#include "gpd_vcloud/CloudSamples.h"
#include "gpd_vcloud/CloudSources.h"
#include "gpd_vcloud/GraspConfig.h"
#include "gpd_vcloud/GraspConfigList.h"
#include "gpd_vcloud/SamplesMsg.h"

visualization_msgs::MarkerArray convertToVisualGraspMsg_vcloud(const std::vector<Grasp>& hands,double outer_diameter, double hand_depth, double finger_width, double hand_height, const std::string& frame_id, const std::vector<double>& scores, double r, double g, double b, double a);
visualization_msgs::Marker createFingerMarker_vcloud(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id, double score, double r, double g, double b, double a);
visualization_msgs::Marker createHandBaseMarker_vcloud(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id, double score, double r, double g, double b, double a);
visualization_msgs::MarkerArray convertToVisualGraspMsg(const std::vector<Grasp>& hands,double outer_diameter, double hand_depth, double finger_width, double hand_height, const std::string& frame_id, double r, double g, double b, double a);
visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id, double r, double g, double b, double a);
visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id, double r, double g, double b, double a);
gpd_vcloud::GraspConfigList createGraspListMsg(const std::vector<Grasp>& hands);
gpd_vcloud::GraspConfig convertToGraspMsg(const Grasp& hand);
visualization_msgs::MarkerArray VisualGraspMsg_deleteAll();

gpd_vcloud::GraspConfig convertToGraspMsg(const Grasp& hand)
{
  gpd_vcloud::GraspConfig msg;
  tf::pointEigenToMsg(hand.getGraspBottom(), msg.bottom);
  tf::pointEigenToMsg(hand.getGraspTop(), msg.top);
  tf::pointEigenToMsg(hand.getGraspSurface(), msg.surface);
  tf::vectorEigenToMsg(hand.getApproach(), msg.approach);
  tf::vectorEigenToMsg(hand.getBinormal(), msg.binormal);
  tf::vectorEigenToMsg(hand.getAxis(), msg.axis);
  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();
  tf::pointEigenToMsg(hand.getSample(), msg.sample);


  Eigen::Vector3d base = hand.getGraspBottom() - 0.00 * hand.getApproach();
  geometry_msgs::Point base_point;
  tf::pointEigenToMsg(base, base_point);
  geometry_msgs::Point position = base_point;
  geometry_msgs::Quaternion orientation;
  Eigen::Quaterniond quat(hand.getFrame());
  orientation.x = quat.x();
  orientation.y = quat.y();
  orientation.z = quat.z();
  orientation.w = quat.w();

  msg.pose.position = position;
  msg.pose.orientation = orientation;

  return msg;
}

gpd_vcloud::GraspConfigList createGraspListMsg(const std::vector<Grasp>& hands)
{
  gpd_vcloud::GraspConfigList msg;

  for (int i = 0; i < hands.size(); i++)
    msg.grasps.push_back(convertToGraspMsg(hands[i]));

  msg.header.stamp = ros::Time::now();// = cloud_camera_header_;
  msg.header.frame_id = "base_link_kinova";

  return msg;
}

visualization_msgs::MarkerArray VisualGraspMsg_deleteAll()
{
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;
	marker.action = visualization_msgs::Marker::DELETEALL;
	marker_array.markers.push_back(marker);
	return marker_array;
}



visualization_msgs::MarkerArray convertToVisualGraspMsg_vcloud(const std::vector<Grasp>& hands,double outer_diameter, double hand_depth, double finger_width, double hand_height, const std::string& frame_id, const std::vector<double>& scores, double r, double g, double b, double a)
{
  double width = outer_diameter;
  double hw = 0.5 * width;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker left_finger, right_finger, base, approach;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center,
    base_center;

  for (int i = 0; i < hands.size(); i++)
  {
    left_bottom = hands[i].getGraspBottom() - (hw - 0.5*finger_width) * hands[i].getBinormal();
    right_bottom = hands[i].getGraspBottom() + (hw - 0.5*finger_width) * hands[i].getBinormal();
    left_top = left_bottom + hand_depth * hands[i].getApproach();
    right_top = right_bottom + hand_depth * hands[i].getApproach();
    left_center = left_bottom + 0.5*(left_top - left_bottom);
    right_center = right_bottom + 0.5*(right_top - right_bottom);
    base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*hands[i].getApproach();
    approach_center = base_center - 0.03*hands[i].getApproach();

    base = createHandBaseMarker_vcloud(left_bottom, right_bottom, hands[i].getFrame(), 0.02, hand_height, i, frame_id, scores[i], r,g,b,a);
    left_finger = createFingerMarker_vcloud(left_center, hands[i].getFrame(), hand_depth, finger_width, hand_height, i*3, frame_id, scores[i], r,g,b,a);
    right_finger = createFingerMarker_vcloud(right_center, hands[i].getFrame(), hand_depth, finger_width, hand_height, i*3+1, frame_id, scores[i], r,g,b,a);
    approach = createFingerMarker_vcloud(approach_center, hands[i].getFrame(), 0.08, finger_width, hand_height, i*3+2, frame_id, scores[i], r,g,b,a);

    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
  }

  return marker_array;
}


visualization_msgs::Marker createFingerMarker_vcloud(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id, double score, double r, double g, double b, double a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(600);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = width * 0.2; // hand closing direction
  marker.scale.z = height * 0.2; // hand vertical direction

  marker.color.a = a;//1;// * score;
  marker.color.r = r;//0;
  marker.color.g = g;//1;
  marker.color.b = b;//1;

  return marker;
}


visualization_msgs::Marker createHandBaseMarker_vcloud(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id, double score, double r, double g, double b, double a)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(600);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length * 0.2; // forward direction
  marker.scale.y = (end - start).norm(); // hand closing direction
  marker.scale.z = height * 0.2; // hand vertical direction

  marker.color.a = a;//1;// * score;
  marker.color.r = r;//0;
  marker.color.g = g;//1;
  marker.color.b = b;//1;

  return marker;
}










visualization_msgs::MarkerArray convertToVisualGraspMsg(const std::vector<Grasp>& hands,double outer_diameter, double hand_depth, double finger_width, double hand_height, const std::string& frame_id, double r, double g, double b, double a)
{
  double width = outer_diameter;
  double hw = 0.5 * width;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker left_finger, right_finger, base, approach;
  Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center,
    base_center;

  for (int i = 0; i < hands.size(); i++)
  {
    left_bottom = hands[i].getGraspBottom() - (hw - 0.5*finger_width) * hands[i].getBinormal();
    right_bottom = hands[i].getGraspBottom() + (hw - 0.5*finger_width) * hands[i].getBinormal();
    left_top = left_bottom + hand_depth * hands[i].getApproach();
    right_top = right_bottom + hand_depth * hands[i].getApproach();
    left_center = left_bottom + 0.5*(left_top - left_bottom);
    right_center = right_bottom + 0.5*(right_top - right_bottom);
    base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*hands[i].getApproach();
    approach_center = base_center - 0.03*hands[i].getApproach();

    base = createHandBaseMarker(left_bottom, right_bottom, hands[i].getFrame(), 0.02, hand_height, i, frame_id, r,g,b,a);
    left_finger = createFingerMarker(left_center, hands[i].getFrame(), hand_depth, finger_width, hand_height, i*3, frame_id, r,g,b,a);
    right_finger = createFingerMarker(right_center, hands[i].getFrame(), hand_depth, finger_width, hand_height, i*3+1, frame_id, r,g,b,a);
    approach = createFingerMarker(approach_center, hands[i].getFrame(), 0.08, finger_width, hand_height, i*3+2, frame_id, r,g,b,a);

    marker_array.markers.push_back(left_finger);
    marker_array.markers.push_back(right_finger);
    marker_array.markers.push_back(approach);
    marker_array.markers.push_back(base);
  }

  return marker_array;
}


visualization_msgs::Marker createFingerMarker(const Eigen::Vector3d& center, const Eigen::Matrix3d& frame, double length, double width, double height, int id, const std::string& frame_id, double r, double g, double b, double a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "finger";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(600);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = length; // forward direction
  marker.scale.y = 0.1 * width; // hand closing direction
  marker.scale.z = 0.1 * height; // hand vertical direction

  marker.color.a = a;//0.5;
  marker.color.r = r;//0;
  marker.color.g = g;//0;
  marker.color.b = b;//1;

  return marker;
}


visualization_msgs::Marker createHandBaseMarker(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Matrix3d& frame, double length, double height, int id, const std::string& frame_id, double r, double g, double b, double a)
{
  Eigen::Vector3d center = start + 0.5 * (end - start);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "hand_base";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = center(0);
  marker.pose.position.y = center(1);
  marker.pose.position.z = center(2);
  marker.lifetime = ros::Duration(600);

  // use orientation of hand frame
  Eigen::Quaterniond quat(frame);
  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();

  // these scales are relative to the hand frame (unit: meters)
  marker.scale.x = 0.1 * length; // forward direction
  marker.scale.y = (end - start).norm(); // hand closing direction
  marker.scale.z = 0.1 * height; // hand vertical direction

  marker.color.a = a;//0.5;
  marker.color.r = r;//0;
  marker.color.g = g;//0;
  marker.color.b = b;//1;

  return marker;
}

#endif /* GRASP_VISUAL_MARKERS_H_ */
