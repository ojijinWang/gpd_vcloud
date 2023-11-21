#include "grasp_detector_vcloud.h"
#include "sub_plot.h"


GraspDetector::GraspDetector(ros::NodeHandle& node)
{
  Eigen::initParallel();

  // Read hand geometry parameters.
  node.param("finger_width", hand_search_params.finger_width_, 0.01);
  node.param("hand_outer_diameter", hand_search_params.hand_outer_diameter_, 0.09);
  node.param("hand_depth", hand_search_params.hand_depth_, 0.06);
  node.param("hand_height", hand_search_params.hand_height_, 0.02);
  node.param("init_bite", hand_search_params.init_bite_, 0.015);
  outer_diameter_ = hand_search_params.hand_outer_diameter_;

  // Read local hand search parameters.
  node.param("nn_radius", hand_search_params.nn_radius_frames_, 0.01);
  node.param("num_orientations", hand_search_params.num_orientations_, 8);
  node.param("num_samples", hand_search_params.num_samples_, 500);
  node.param("num_threads", hand_search_params.num_threads_, 1);
  node.param("rotation_axis", hand_search_params.rotation_axis_, 2); // cannot be changed

  // Read plotting parameters.
  node.param("plot_samples", plot_samples_, false);
  node.param("plot_normals", plot_normals_, false);
  generator_params.plot_normals_ = plot_normals_;
  node.param("plot_filtered_grasps", plot_filtered_grasps_, false);
  node.param("plot_valid_grasps", plot_valid_grasps_, false);
  node.param("plot_clusters", plot_clusters_, false);
  node.param("plot_selected_grasps", plot_selected_grasps_, false);

  // Read general parameters.
  generator_params.num_samples_ = hand_search_params.num_samples_;
  generator_params.num_threads_ = hand_search_params.num_threads_;
  node.param("plot_candidates", generator_params.plot_grasps_, false);

  // Read preprocessing parameters.
  node.param("remove_outliers", generator_params.remove_statistical_outliers_, true);
  node.param("voxelize", generator_params.voxelize_, true);
  node.getParam("workspace", generator_params.workspace_);
  node.getParam("workspace_grasps", workspace_);

  // Create object to generate grasp candidates.
  candidates_generator_ = new CandidatesGenerator(generator_params, hand_search_params);

  // Read classification parameters and create classifier.
  std::string model_file, weights_file;
  bool use_gpu;
  node.param("model_file", model_file, std::string(""));
  node.param("trained_file", weights_file, std::string(""));
  node.param("min_score_diff", min_score_diff_, 500.0);
  node.param("create_image_batches", create_image_batches_, true);
  node.param("use_gpu", use_gpu, true);
  classifier_ = new CaffeClassifier(model_file, weights_file, use_gpu);

  // Read grasp image parameters.
  node.param("image_outer_diameter", image_params_.outer_diameter_, hand_search_params.hand_outer_diameter_);
  node.param("image_depth", image_params_.depth_, hand_search_params.hand_depth_);
  node.param("image_height", image_params_.height_, hand_search_params.hand_height_);
  node.param("image_size", image_params_.size_, 60);
  node.param("image_num_channels", image_params_.num_channels_, 15);

  // Read learning parameters.
  bool remove_plane;
  node.param("remove_plane_before_image_calculation", remove_plane, false);

  // Create object to create grasp images from grasp candidates (used for classification)
  learning_ = new Learning(image_params_, hand_search_params.num_threads_, hand_search_params.num_orientations_, false, remove_plane);
  image_params_1channel_ = image_params_;
  image_params_1channel_.num_channels_ = 1;
  learning_1channel_ = new Learning(image_params_1channel_, hand_search_params.num_threads_, hand_search_params.num_orientations_, false, remove_plane);

  // Read grasp filtering parameters
  node.param("filter_grasps", filter_grasps_, false);
  node.param("filter_half_antipodal", filter_half_antipodal_, false);
  std::vector<double> gripper_width_range(2);
  gripper_width_range[0] = 0.03;
  gripper_width_range[1] = 0.07;
  node.getParam("gripper_width_range", gripper_width_range);
  min_aperture_ = gripper_width_range[0];
  max_aperture_ = gripper_width_range[1];

  // Read clustering parameters
  int min_inliers;
  node.param("min_inliers", min_inliers, 0);
  clustering_ = new Clustering(min_inliers);
  cluster_grasps_ = min_inliers > 0 ? true : false;

  // Read grasp selection parameters
  node.param("num_selected", num_selected_, 100);

  // vcloudMap Param
  node.param("resolution", vcloud_resolution_, 0.005);
}






bool GraspDetector::detectGrasps(const std::vector<double>& viewpoint, const pcl::PointCloud<pcl::PointXYZ>& cloud_occupy, const pcl::PointCloud<pcl::PointXYZ>& cloud_empty, std::vector<Grasp>& selected_grasps, std::vector<double>& scores, std::vector<Grasp>& candidate_grasps, std::vector<Grasp>& candidate_grasps_2)
{
	ros::WallTime startTime = ros::WallTime::now();
	selected_grasps.clear();
	scores.clear();

	PointCloudRGBA cloud;
	PointCloudRGBA cloud_clear;
	pcl::copyPointCloud(cloud_occupy,	cloud);
	pcl::copyPointCloud(cloud_empty,	cloud_clear);
	PointCloudRGBA::Ptr cloud_ptr(new PointCloudRGBA);
	PointCloudRGBA::Ptr cloud_clear_ptr(new PointCloudRGBA);
	*cloud_ptr = cloud;
	*cloud_clear_ptr = cloud_clear;

	// Load point cloud
	Eigen::Matrix3Xd view_points(3,1);
	view_points << viewpoint[0], viewpoint[1], viewpoint[2];
	CloudCamera cloud_cam_1(cloud_ptr, 0, view_points);
	if (cloud_cam_1.getCloudOriginal()->size() == 0)
	{ ROS_ERROR("Point cloud is empty!"); return false; }

	CloudCamera cloud_cam_2(cloud_clear_ptr, 0, view_points);
	if (cloud_cam_2.getCloudOriginal()->size() == 0)
	{ ROS_ERROR("Point cloud clear is empty!"); return false; }
	ROS_WARN("Load a point cloud");

  	ROS_WARN("gpd Time, preoperation, %.4f", (ros::WallTime::now() - startTime).toSec());

	cloud_cam_occupy = cloud_cam_1;
	cloud_cam_empty = cloud_cam_2;
	candidates_generator_->preprocessPointCloud(cloud_cam_occupy);
	candidates_generator_->preprocessPointCloud(cloud_cam_empty);

	// 1. Generate grasp candidates.
	std::vector<GraspSet> candidates = candidates_generator_->generateGraspCandidateSets(cloud_cam_occupy);
	//ROS_INFO_STREAM("Generated " << candidates.size() << " grasp candidate sets.");
	if (candidates.size() == 0)
	{ ROS_ERROR("No grasp candidates found!"); return false; }
	// 1.1 show results
	createImages_vcloud(cloud_cam_occupy, cloud_cam_empty, candidates);
  	ROS_WARN("gpd Time, candidate, %.4f", (ros::WallTime::now() - startTime).toSec());


	std::vector<Grasp> grasp_temp_0 = extractHypotheses(candidates);
	ROS_WARN("gpd Time, candidate filter total, %d", (int)grasp_temp_0.size());
	// 2 Filter grasps.
	// 2.1 Filter workspace.
	candidates = filterGraspsWorkspace(candidates, workspace_);
	candidate_grasps = extractHypotheses(candidates);
	// 2.2 Filter half grasps.
	//candidates = filterHalfAntipodal(candidates);
	// 2.3 Filter Shadow
	filter_candidate_vcloud(cloud_empty, candidates);
	candidate_grasps_2 = extractHypotheses(candidates);

	std::vector<Grasp> grasp_temp_3 = extractHypotheses(candidates);
	ROS_WARN("gpd Time, candidate filter both, %d", (int)grasp_temp_3.size());

	// 3. Classify each grasp candidate. (Note: switch from a list of hypothesis sets to a list of grasp hypotheses)
	std::vector<Grasp> valid_grasps = classifyGraspCandidates(cloud_cam_occupy, candidates);
	ROS_INFO_STREAM("Predicted " << valid_grasps.size() << " valid grasps.");

  	ROS_WARN("gpd Time, candidate classfy, %.4f", (ros::WallTime::now() - startTime).toSec());

	if (valid_grasps.size() == 0)
	{ std::cout << "Not enough valid grasps predicted! Using all grasps from previous step.\n"; return false; }

	// 4. Cluster the grasps. //TODO we dont do the cluster here.
	std::vector<Grasp> clustered_grasps;
	//clustered_grasps = valid_grasps;
	if (cluster_grasps_)
	{
		clustered_grasps = findClusters(valid_grasps);
		 ROS_INFO_STREAM("Found " << clustered_grasps.size() << " clusters.");
		if (clustered_grasps.size() <= 3)
		{
			std::cout << "Not enough clusters found! Using all grasps from previous step.\n";
			clustered_grasps = valid_grasps;
		}
	}else{
		clustered_grasps = valid_grasps;
	}

	// 5. Select highest-scoring grasps.
	if (clustered_grasps.size() > num_selected_)
	{
		std::cout << "Partial Sorting the grasps based on their score ... \n";
		std::partial_sort(clustered_grasps.begin(), clustered_grasps.begin() + num_selected_, clustered_grasps.end(), isScoreGreater);
		selected_grasps.assign(clustered_grasps.begin(), clustered_grasps.begin() + num_selected_);
	}
	else
	{
		std::cout << "Sorting the grasps based on their score ... \n";
		std::sort(clustered_grasps.begin(), clustered_grasps.end(), isScoreGreater);
		selected_grasps = clustered_grasps;
	}
	for (int i = 0; i < selected_grasps.size(); i++)
	{ std::cout << "Grasp " << i << ": " << selected_grasps[i].getScore() << "\n"; }
  	ROS_INFO_STREAM("Selected the " << selected_grasps.size() << " highest scoring grasps.");

	ROS_WARN("gpd Time, sort result, %.4f", (ros::WallTime::now() - startTime).toSec());


	// 6. Discoveries & score
	std::vector<cv::Mat> image_list_occupy;
	std::vector<cv::Mat> image_list_empty;
	std::vector<cv::Mat> image_list_vcloud;
	createImages_vcloud(cloud_cam_occupy, cloud_cam_empty, selected_grasps, image_list_occupy, image_list_empty, image_list_vcloud);
	std::vector<cv::Mat> image_list_visibility = createDiscoveriesImages(image_list_occupy);
	scores =  GraspDetector::getScoreDiscoveriesImages(image_list_vcloud, image_list_visibility);

	return true;
}

std::vector<cv::Mat> GraspDetector::createImages_vcloud(CloudCamera& cloud_occupy, CloudCamera& cloud_empty, const std::vector<Grasp>& handes, std::vector<cv::Mat>& image_list_occupy, std::vector<cv::Mat>& image_list_empty, std::vector<cv::Mat>& image_list_vcloud)
{
	std::vector<GraspSet> candidates((int)handes.size());
	Eigen::Array<bool, 1, Eigen::Dynamic> isValid = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, true);
	for(int i=0; i<(int)handes.size(); i++)
	{
		std::vector<Grasp> hande1(1);
		hande1[0] = handes[i];
		candidates[i].setHands(hande1);
		candidates[i].setIsValid(isValid);
		candidates[i].setSample(handes[i].getGraspBottom());
	}
	
	// general images.
  	learning_valid_1channel_ = new Learning(image_params_1channel_, 4, 1, false, false);
	// images of occupy
	image_list_occupy = learning_valid_1channel_->createImages(cloud_occupy, candidates);
	// images of empty
	image_list_empty =  learning_valid_1channel_->createImages(cloud_empty, candidates);

	int rown = (int)sqrt(candidates.size());
	cv::Mat candidates_images_occupy = makeCanvas(image_list_occupy, 70 * rown, rown);
	cv::Mat candidates_images_empty = makeCanvas(image_list_empty, 70 * rown, rown);

	image_list_vcloud.resize((int)image_list_occupy.size());
	for(int i=0; i<(int)image_list_occupy.size(); i++)
	{ image_list_vcloud[i] = image_list_occupy[i] + image_list_empty[i]*(100.0 / 255.0); }
	cv::Mat candidates_images_vcloud = makeCanvas(image_list_vcloud, 70 * rown, rown);

	//cv::imshow("candidates_images_occupy", candidates_images_occupy);
	//cv::imshow("candidates_images_empty", candidates_images_empty);
	//cv::imshow("candidates_images_vcloud", candidates_images_vcloud);

	return image_list_vcloud;
}



std::vector<cv::Mat> GraspDetector::createImages_vcloud(CloudCamera& cloud_occupy, CloudCamera& cloud_empty, const std::vector<GraspSet>& candidates)
{
	// general images.
	std::vector<cv::Mat> image_list_occupy = learning_1channel_->createImages(cloud_occupy, candidates);
	image_list_occupy = learning_1channel_->createImages(cloud_occupy, candidates);	//agian, there are some mistake
	//cv::Mat candidates_images_occupy = makeCanvas(image_list_occupy, 70 * (int)candidates.size(), (int)candidates.size());
	// images of empty
	std::vector<cv::Mat> image_list_empty =  learning_1channel_->createImages(cloud_empty, candidates);
	//cv::Mat candidates_images_empty = makeCanvas(image_list_empty, 70 * (int)candidates.size(), (int)candidates.size());


	std::vector<cv::Mat> image_list_vcloud((int)image_list_occupy.size());
	for(int i=0; i<(int)image_list_occupy.size(); i++)
	{
		image_list_vcloud[i] = image_list_occupy[i] + image_list_empty[i]*(100.0 / 255.0);
	}

	// *** save image
	//cv::Mat candidates_images_vcloud = makeCanvas(image_list_vcloud, 70 * (int)candidates.size(), (int)candidates.size());
	//imwrite("/home/matsu2/image_1.png",candidates_images_occupy);
	//imwrite("/home/matsu2/image_2.png",candidates_images_empty);
	//imwrite("/home/matsu2/image_3.png",candidates_images_vcloud);
	//ROS_INFO("save grasp images");
	//cv::imshow("candidates_images_occupy", candidates_images_occupy);
	//cv::imshow("candidates_images_empty", candidates_images_empty);
	//cv::imshow("candidates_images_vcloud", candidates_images_vcloud);

	return image_list_vcloud;
}


std::vector<cv::Mat> GraspDetector::createDiscoveriesImages(const std::vector<cv::Mat>& image_list_occupy)
{
	std::vector<cv::Mat> image_list_visibility((int)image_list_occupy.size());
	for(int i=0; i<(int)image_list_occupy.size(); i++)
	{
		image_list_visibility[i] = cv::Mat(image_list_occupy[i].rows, image_list_occupy[i].cols, CV_8UC1, cv::Scalar(0));
		int height = image_list_occupy[i].rows;
		int width = image_list_occupy[i].cols;
		for(int col=0; col<width; col++)
		{
			for(int row=height-1; row>=0; row--)
			{
				image_list_visibility[i].at<unsigned char>(row, col) = 100;
				if(image_list_occupy[i].at<unsigned char>(row, col) != 0)
				{ image_list_visibility[i].at<unsigned char>(row, col) = 0; break; }
			}
		}
	}
	int rown = (int)sqrt(image_list_occupy.size());
	cv::Mat candidates_images_discover = makeCanvas(image_list_visibility, 70 * rown, rown);
	//cv::imshow("candidates_images_discover", candidates_images_discover);

	return image_list_visibility;
}

std::vector<double> GraspDetector::getScoreDiscoveriesImages(const std::vector<cv::Mat>& image_list_vcloud, const std::vector<cv::Mat>& image_list_visibility)
{
	if(image_list_vcloud.size() != image_list_visibility.size())
	{ ROS_WARN("error in getScoreDiscoveriesImages(): size differ"); }
	
	std::vector<double> scores((int)image_list_vcloud.size());
	for(int i=0; i<(int)image_list_vcloud.size(); i++)
	{
		scores[i] = 0;
		double count_disovery = 0;
		double count_same = 0;
		for (int row=0; row<image_list_vcloud[i].rows; row++)
		{
			const unsigned char *data1 = image_list_vcloud[i].ptr<unsigned char>(row);
			const unsigned char *data2 = image_list_visibility[i].ptr<unsigned char>(row);
			for (int col=0; col<image_list_vcloud[i].cols; col++)
			{
				unsigned char data_1 = data1[col];
				unsigned char data_2 = data2[col];
				if(data_2 > 0 && data_2 < 101)
				{ 
					count_disovery++; 
					if(data_1 > 0 && data_1 < 101)
					{ count_same += abs(100 - (data_2 - data_1)) / 100.0; }
				}
			}
		}
		if(count_disovery!=0 && count_same!=0 )
		{ scores[i] = count_same/count_disovery;  }
		ROS_WARN("%.2f, %.2f, %.2f",count_disovery, count_same, scores[i]);
	}
	return scores;
}



//the candidate point & finger position should be covered by empty points. 8 points in range
void GraspDetector::filter_candidate_vcloud(const pcl::PointCloud<pcl::PointXYZ>& pclCloud_clear, std::vector<GraspSet>& candidates)
{
        const HandSearch::Parameters& params = candidates_generator_->getHandSearchParams();
	double finger_width = params.finger_width_;
	double hand_depth = params.hand_depth_;
  	double hw = 0.08;
	double resolution_ = vcloud_resolution_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud = pclCloud_clear;
	//define KDtree
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_left;
	kdtree_left.setInputCloud (cloud);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_right;
	kdtree_right.setInputCloud (cloud);

	int count_total = 0;
	int count_remain = 0;
	for(int i=0; i<candidates.size(); i++)
	{
		std::vector<Grasp> hands = candidates[i].getHypotheses();
		Eigen::Array<bool, 1, Eigen::Dynamic> isValid = candidates[i].getIsValid();
		for(int j=0; j<hands.size(); j++)
		{
			if(isValid[j])
			{
				count_total++;
				Eigen::Vector3d left_bottom, right_bottom, left_top, right_top, left_center, right_center, approach_center, base_center;
				left_bottom = hands[j].getGraspBottom() - (hw - 0.5*finger_width) * hands[j].getBinormal();
				right_bottom = hands[j].getGraspBottom() + (hw - 0.5*finger_width) * hands[j].getBinormal();
				left_top = left_bottom + hand_depth * hands[j].getApproach();
				right_top = right_bottom + hand_depth * hands[j].getApproach();
				left_center = left_bottom + 0.5 * hand_depth * hands[j].getApproach();
				right_center = right_bottom + 0.5 * hand_depth * hands[j].getApproach();
				base_center = left_bottom + 0.5*(right_bottom - left_bottom) - 0.01*hands[j].getApproach();
				approach_center = base_center - 0.005*hands[j].getApproach();

				Eigen::Vector3d left_tip, right_tip;
				left_tip = left_bottom + 0.75 * hand_depth * hands[j].getApproach();
				right_tip = right_bottom + 0.75 * hand_depth * hands[j].getApproach();
				Eigen::Vector3d left_tip_2, right_tip_2;
				left_tip_2 = left_bottom + 1.0 * hand_depth * hands[j].getApproach();
				right_tip_2 = right_bottom + 1.0 * hand_depth * hands[j].getApproach();

				pcl::PointXYZ searchPoint;
				searchPoint.x = approach_center(0);
				searchPoint.y = approach_center(1);
				searchPoint.z = approach_center(2);
				//ROS_WARN("%.4f, %.4f, %.4f", searchPoint.x, searchPoint.y, searchPoint.z);

				int K = 10;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				//handle should be in light			
				if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
				{
					int count = 0;
					float radius = resolution_ * 1.5;
					float radius2 = radius*radius;
					for (size_t k = 0; k < pointIdxNKNSearch.size (); k++)
					{
						//double x = cloud->points[ pointIdxNKNSearch[k] ].x;
                				//double y = cloud->points[ pointIdxNKNSearch[k] ].y;
						//double z = cloud->points[ pointIdxNKNSearch[k] ].z;
						float r2 = pointNKNSquaredDistance[k];
						if(r2 < radius2)
						{ count ++; }
					}
					if(count < 4)
					{ isValid[j] = false; continue; }
				}

				bool is_left_light = true;
				bool is_right_light = true;
				//left finger should be in light
				int K_left = 10;
				std::vector<int> pointIdxNKNSearch_left(K_left);
				std::vector<float> pointNKNSquaredDistance_left(K_left);
				searchPoint.x = left_center(0);
				searchPoint.y = left_center(1);
				searchPoint.z = left_center(2);
				if ( kdtree_left.nearestKSearch (searchPoint, K_left, pointIdxNKNSearch_left, pointNKNSquaredDistance_left) > 0 )
				{
					int count = 0;
					float radius = resolution_ * 1.5;
					float radius2 = radius*radius;
					for (size_t k = 0; k < pointIdxNKNSearch_left.size (); k++)
					{
						float r2 = pointNKNSquaredDistance_left[k];
						if(r2 < radius2)
						{ count ++; }
					}
					if(count < 1)
					{ is_left_light = false; }
				}
				//right finger should be in light
				int K_right = 10;
				std::vector<int> pointIdxNKNSearch_right(K_right);
				std::vector<float> pointNKNSquaredDistance_right(K_right);
				searchPoint.x = right_center(0);
				searchPoint.y = right_center(1);
				searchPoint.z = right_center(2);
				if ( kdtree_right.nearestKSearch (searchPoint, K_right, pointIdxNKNSearch_right, pointNKNSquaredDistance_right) > 0 )
				{
					int count = 0;
					float radius = resolution_ * 1.5;
					float radius2 = radius*radius;
					for (size_t k = 0; k < pointIdxNKNSearch_right.size (); k++)
					{
						float r2 = pointNKNSquaredDistance_right[k];
						if(r2 < radius2)
						{ count ++; }
					}
					if(count < 1)
					{ is_right_light = false; }
				}
				if(is_left_light && is_right_light)//first class
				{ ; }else
				{ isValid[j] = false; continue; }

				count_remain++;
			}
		}
		candidates[i].setIsValid(isValid);
	}
	ROS_WARN("candinates ramian %d : %d", count_total, count_remain);
}





std::vector<Grasp> GraspDetector::classifyGraspCandidates(const CloudCamera& cloud_cam, std::vector<GraspSet>& candidates)
{
  // Create a grasp image for each grasp candidate.
  double t0 = omp_get_wtime();
  std::cout << "Creating grasp images for classifier input ...\n";
  std::vector<float> scores;
  std::vector<Grasp> grasp_list;
  int num_orientations = candidates[0].getHypotheses().size();

  // Create images in batches if required (less memory usage).
  if (create_image_batches_)
  {
    ROS_WARN("create_image_batches_");
    int batch_size = classifier_->getBatchSize();
    int num_iterations = (int) ceil(candidates.size() * num_orientations / (double) batch_size);
    int step_size = (int) floor(batch_size / (double) num_orientations);
    std::cout << " num_iterations: " << num_iterations << ", step_size: " << step_size << "\n";

    // Process the grasp candidates in batches.
    for (int i = 0; i < num_iterations; i++)
    {
      std::cout << i << "\n";
      std::vector<GraspSet>::iterator start = candidates.begin() + i * step_size;
      std::vector<GraspSet>::iterator stop;
      if (i < num_iterations - 1)
      {
        stop = candidates.begin() + i * step_size + step_size;
      }
      else
      {
        stop = candidates.end();
      }

      std::vector<GraspSet> hand_set_sublist(start, stop);
      //std::vector<cv::Mat> image_list = learning_->createImages(cloud_cam, hand_set_sublist);
	std::vector<cv::Mat> image_list;
	learning_->createImages(cloud_cam, hand_set_sublist, image_list);

      std::vector<Grasp> valid_grasps;
      std::vector<cv::Mat> valid_images;
      extractGraspsAndImages(candidates, image_list, valid_grasps, valid_images);

      std::vector<float> scores_sublist = classifier_->classifyImages(valid_images);
      scores.insert(scores.end(), scores_sublist.begin(), scores_sublist.end());
      grasp_list.insert(grasp_list.end(), valid_grasps.begin(), valid_grasps.end());
    }
  }
  else
  {
    ROS_INFO("no create_image_batches");
    // Create the grasp images.
    //std::vector<cv::Mat> image_list = learning_->createImages(cloud_cam, candidates); 
    std::vector<cv::Mat> image_list;
    learning_->createImages(cloud_cam, candidates, image_list);
    std::cout << " Image creation time: " << omp_get_wtime() - t0 << std::endl;

    std::vector<Grasp> valid_grasps;
    std::vector<cv::Mat> valid_images;
    extractGraspsAndImages(candidates, image_list, valid_grasps, valid_images);

    // Classify the grasp images.
    double t0_prediction = omp_get_wtime();
    scores = classifier_->classifyImages(valid_images);
    grasp_list.assign(valid_grasps.begin(), valid_grasps.end());
    std::cout << " Prediction time: " << omp_get_wtime() - t0 << std::endl;
  }

  // Select grasps with a score of at least <min_score_diff_>.
  std::vector<Grasp> valid_grasps;
  for (int i = 0; i < grasp_list.size(); i++)
  {
    if (scores[i] >= min_score_diff_)
    {
      std::cout << "grasp #" << i << ", score: " << scores[i] << "\n";
      valid_grasps.push_back(grasp_list[i]);
      valid_grasps[valid_grasps.size() - 1].setScore(scores[i]);
      valid_grasps[valid_grasps.size() - 1].setFullAntipodal(true);
    }
  }

  std::cout << "Found " << valid_grasps.size() << " grasps with a score >= " << min_score_diff_ << "\n";
  std::cout << "Total classification time: " << omp_get_wtime() - t0 << std::endl;

  if (plot_valid_grasps_)
  {
    Plot plotter;
    const HandSearch::Parameters& params = candidates_generator_->getHandSearchParams();
    plotter.plotFingers3D(valid_grasps, cloud_cam.getCloudOriginal(), "Valid Grasps", params.hand_outer_diameter_,
      params.finger_width_, params.hand_depth_, params.hand_height_);
  }

  return valid_grasps;
}


std::vector<GraspSet> GraspDetector::filterGraspsWorkspace(const std::vector<GraspSet>& hand_set_list, const std::vector<double>& workspace)
{
  int remaining = 0;
  std::vector<GraspSet> hand_set_list_out;

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid = hand_set_list[i].getIsValid();

    for (int j = 0; j < hands.size(); j++)
    {
      if (is_valid(j))
      {
        double half_width = 0.5 * outer_diameter_;
        Eigen::Vector3d left_bottom = hands[j].getGraspBottom() + half_width * hands[j].getBinormal();
        Eigen::Vector3d right_bottom = hands[j].getGraspBottom() - half_width * hands[j].getBinormal();
        Eigen::Vector3d left_top = hands[j].getGraspTop() + half_width * hands[j].getBinormal();
        Eigen::Vector3d right_top = hands[j].getGraspTop() - half_width * hands[j].getBinormal();
        Eigen::Vector3d approach = hands[j].getGraspBottom() - 0.05 * hands[j].getApproach();
        Eigen::VectorXd x(5), y(5), z(5);
        x << left_bottom(0), right_bottom(0), left_top(0), right_top(0), approach(0);
        y << left_bottom(1), right_bottom(1), left_top(1), right_top(1), approach(1);
        z << left_bottom(2), right_bottom(2), left_top(2), right_top(2), approach(2);
        double aperture = hands[j].getGraspWidth();

        if (aperture >= min_aperture_ && aperture <= max_aperture_ // make sure the object fits into the hand
          && x.minCoeff() >= workspace[0] && x.maxCoeff() <= workspace[1] // avoid grasping outside the x-workspace
          && y.minCoeff() >= workspace[2] && y.maxCoeff() <= workspace[3] // avoid grasping outside the y-workspace
          && z.minCoeff() >= workspace[4] && z.maxCoeff() <= workspace[5]) // avoid grasping outside the z-workspace
        {
          is_valid(j) = true;
          remaining++;
        }
        else
        {
          is_valid(j) = false;
        }
      }
    }

    if (is_valid.any())
    {
      hand_set_list_out.push_back(hand_set_list[i]);
      hand_set_list_out[hand_set_list_out.size() - 1].setIsValid(is_valid);
    }
  }

  ROS_INFO_STREAM("# grasps within workspace and gripper width: " << remaining);

  return hand_set_list_out;
}

std::vector<GraspSet> GraspDetector::filterFullAntipodal(const std::vector<GraspSet>& hand_set_list)
{
  int remaining = 0;
  std::vector<GraspSet> hand_set_list_out;

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid = hand_set_list[i].getIsValid();

    for (int j = 0; j < hands.size(); j++)
    {
      if (is_valid(j))
      {
        if ( hands[j].isFullAntipodal())
        {
          is_valid(j) = true;
          remaining++;
        }
        else
        {
          is_valid(j) = false;
        }
      }
    }

    if (is_valid.any())
    {
      hand_set_list_out.push_back(hand_set_list[i]);
      hand_set_list_out[hand_set_list_out.size() - 1].setIsValid(is_valid);
    }
  }

  ROS_INFO_STREAM("# grasps that are not half-antipodal: " << remaining);

  return hand_set_list_out;
}

std::vector<GraspSet> GraspDetector::filterHalfAntipodal(const std::vector<GraspSet>& hand_set_list)
{
  int remaining = 0;
  std::vector<GraspSet> hand_set_list_out;

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid = hand_set_list[i].getIsValid();

    for (int j = 0; j < hands.size(); j++)
    {
      if (is_valid(j))
      {
        if (!hands[j].isHalfAntipodal() || hands[j].isFullAntipodal())
        {
          is_valid(j) = true;
          remaining++;
        }
        else
        {
          is_valid(j) = false;
        }
      }
    }

    if (is_valid.any())
    {
      hand_set_list_out.push_back(hand_set_list[i]);
      hand_set_list_out[hand_set_list_out.size() - 1].setIsValid(is_valid);
    }
  }

  ROS_INFO_STREAM("# grasps that are not half-antipodal: " << remaining);

  return hand_set_list_out;
}


std::vector<Grasp> GraspDetector::extractHypotheses(const std::vector<GraspSet>& hand_set_list)
{
  std::vector<Grasp> hands_out;
  hands_out.resize(0);

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();

    for (int j = 0; j < hands.size(); j++)
    {
      if (hand_set_list[i].getIsValid()(j))
      {
        hands_out.push_back(hands[j]);
      }
    }
  }

  return hands_out;
}

void GraspDetector::extractGraspsAndImages(const std::vector<GraspSet>& hand_set_list, const std::vector<cv::Mat>& images, std::vector<Grasp>& grasps_out, std::vector<cv::Mat>& images_out)
{
  grasps_out.resize(0);
  images_out.resize(0);
  int num_orientations = hand_set_list[0].getHypotheses().size();

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();

    for (int j = 0; j < hands.size(); j++)
    {
      if (hand_set_list[i].getIsValid()(j))
      {	
        grasps_out.push_back(hands[j]);
        images_out.push_back(images[i * num_orientations + j]);
      }
    }
  }
}

void GraspDetector::extractGraspsAndImages_vcloud(const std::vector<GraspSet>& hand_set_list, const std::vector<cv::Mat>& images, const std::vector<cv::Mat>& images_occupy, const std::vector<cv::Mat>& images_empty, std::vector<Grasp>& grasps_out, std::vector<cv::Mat>& images_out, std::vector<cv::Mat>& images_occupy_out, std::vector<cv::Mat>& images_empty_out)
{
  grasps_out.resize(0);
  images_out.resize(0);
  images_occupy_out.resize(0);
  images_empty_out.resize(0);
  int num_orientations = hand_set_list[0].getHypotheses().size();

  for (int i = 0; i < hand_set_list.size(); i++)
  {
    const std::vector<Grasp>& hands = hand_set_list[i].getHypotheses();

    for (int j = 0; j < hands.size(); j++)
    {
      if (hand_set_list[i].getIsValid()(j))
      {
        grasps_out.push_back(hands[j]);
        images_out.push_back(images[i * num_orientations + j]);
	images_occupy_out.push_back(images_occupy[i * num_orientations + j]);
	images_empty_out.push_back(images_empty[i * num_orientations + j]);
      }
    }
  }
}


std::vector<Grasp> GraspDetector::findClusters(const std::vector<Grasp>& grasps)
{
  return clustering_->findClusters(grasps);
}
