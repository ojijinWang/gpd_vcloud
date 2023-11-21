/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andreas ten Pas
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GRASP_DETECTOR_H_
#define GRASP_DETECTOR_H_


// *** System
#include <algorithm>
#include <vector>
// *** Caffe
#include "caffe/caffe.hpp"
#include "caffe/layers/memory_data_layer.hpp"
#include "caffe/util/io.hpp"
// *** PCL
#include <pcl/common/common.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// *** ROS
#include <ros/ros.h>
// *** Grasp Candidates Generator
#include <gpg/cloud_camera.h>
#include <gpg/candidates_generator.h>
#include <gpg/grasp.h>
#include <gpg/grasp_set.h>
#include <gpg/plot.h>
// *** Custom
#include "../include/gpd_vcloud/caffe_classifier.h"
#include "../include/gpd_vcloud/clustering.h"
#include "../include/gpd_vcloud/learning.h"

class GraspDetector
{
public:

  GraspDetector(ros::NodeHandle& node);

  ~GraspDetector()
  {
    delete candidates_generator_;
    delete learning_;
    delete clustering_;
    delete classifier_;
  }
  
  bool detectGrasps(const std::vector<double>& viewpoint, const pcl::PointCloud<pcl::PointXYZ>& cloud_occupy, const pcl::PointCloud<pcl::PointXYZ>& cloud_empty, std::vector<Grasp>& selected_grasps, std::vector<double>& scores, std::vector<Grasp>& candidate_grasps, std::vector<Grasp>& candidate_grasps_2);

  //Filters
  std::vector<GraspSet> filterGraspsWorkspace(const std::vector<GraspSet>& hand_set_list, const std::vector<double>& workspace);
  std::vector<GraspSet> filterHalfAntipodal(const std::vector<GraspSet>& hand_set_list);
  std::vector<GraspSet> filterFullAntipodal(const std::vector<GraspSet>& hand_set_list);
  void filter_candidate_vcloud(const pcl::PointCloud<pcl::PointXYZ>& pclCloud_clear, std::vector<GraspSet>& candidates);

  //gegerate images
  std::vector<cv::Mat> createImages_vcloud(CloudCamera& cloud_occupy, CloudCamera& cloud_empty, const std::vector<GraspSet>& candidates);
  std::vector<cv::Mat> createImages_vcloud(CloudCamera& cloud_occupy, CloudCamera& cloud_empty, const std::vector<Grasp>& handes);
  std::vector<cv::Mat> createImages_vcloud(CloudCamera& cloud_occupy, CloudCamera& cloud_empty, const std::vector<Grasp>& handes, std::vector<cv::Mat>& image_list_occupy, std::vector<cv::Mat>& image_list_empty, std::vector<cv::Mat>& image_list_vcloud);

  //score vcloudmap
  std::vector<cv::Mat> createDiscoveriesImages(const std::vector<cv::Mat>& image_list_occupy);
  std::vector<double> getScoreDiscoveriesImages(const std::vector<cv::Mat>& image_list_vcloud, const std::vector<cv::Mat>& image_list_visibility);

  std::vector<Grasp> extractHypotheses(const std::vector<GraspSet>& hand_set_list);

  void extractGraspsAndImages(const std::vector<GraspSet>& hand_set_list, const std::vector<cv::Mat>& images, std::vector<Grasp>& grasps_out, std::vector<cv::Mat>& images_out);
  void extractGraspsAndImages_vcloud(const std::vector<GraspSet>& hand_set_list, const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& images_occupy, const std::vector<cv::Mat>& images_empty, std::vector<Grasp>& grasps_out, std::vector<cv::Mat>& images_out, std::vector<cv::Mat>& images_occupy_out, std::vector<cv::Mat>& images_empty_out);

  std::vector<Grasp> classifyGraspCandidates(const CloudCamera& cloud_cam, std::vector<GraspSet>& candidates);

  std::vector<Grasp> findClusters(const std::vector<Grasp>& grasps);

  static bool isScoreGreater(const Grasp& hypothesis1, const Grasp& hypothesis2)
  { return hypothesis1.getScore() > hypothesis2.getScore(); }

  const HandSearch::Parameters& getHandSearchParameters()
  { return candidates_generator_->getHandSearchParams(); }


  CloudCamera cloud_cam_occupy;
  CloudCamera cloud_cam_empty;

  CandidatesGenerator* candidates_generator_; ///< pointer to object for grasp candidate generation
  Learning* learning_; ///< pointer to object for grasp image creation
  Learning* learning_1channel_;
  Learning* learning_valid_1channel_;
  Clustering* clustering_; ///< pointer to object for clustering geometrically aligned grasps
  CaffeClassifier* classifier_; ///< pointer to object for classification of candidates

  Learning::ImageParameters image_params_; // grasp image parameters
  Learning::ImageParameters image_params_1channel_; // grasp image parameters

  CandidatesGenerator::Parameters generator_params;
  HandSearch::Parameters hand_search_params;

  // classification parameters
  double min_score_diff_; ///< minimum classifier confidence score
  bool create_image_batches_; ///< if images are created in batches (reduces memory usage)

  // plotting parameters
  bool plot_normals_; ///< if normals are plotted
  bool plot_samples_; ///< if samples/indices are plotted
  bool plot_filtered_grasps_; ///< if filtered grasps are plotted
  bool plot_valid_grasps_; ///< if positive grasp instances are plotted
  bool plot_clusters_; ///< if grasp clusters are plotted
  bool plot_selected_grasps_; ///< if selected grasps are plotted

  // filtering parameters
  bool filter_grasps_; ///< if grasps are filtered based on the robot's workspace and the robot hand width
  bool filter_half_antipodal_; ///< if grasps are filtered based on being half-antipodal
  bool cluster_grasps_; ///< if grasps are clustered
  double outer_diameter_; ///< the outer diameter of the robot hand
  double min_aperture_; ///< the minimum opening width of the robot hand
  double max_aperture_; ///< the maximum opening width of the robot hand
  std::vector<double> workspace_; ///< the workspace of the robot

  double vcloud_resolution_;

  // selection parameters
  int num_selected_; ///< the number of selected grasps
};

#endif /* GRASP_DETECTOR_H_ */
