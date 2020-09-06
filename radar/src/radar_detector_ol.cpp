// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
// SVM
#include "svm.h"

#include <boost/algorithm/string.hpp> 

typedef struct feature {
  /*** for visualization ***/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  /*** for classification ***/
  int number_points;
  float min_distance;
  Eigen::Matrix3f covariance_3d;
  Eigen::Matrix3f moment_3d;
  float slice[20];
  float intensity[27];
  float velocity;
} Feature;

static const int FEATURE_SIZE = 62;
float svm_range_[FEATURE_SIZE][2];
float svm_xlower_ = -1.0, svm_xupper_ = 1.0;

class Object3dDetector {
private:
  /*** Publishers and Subscribers ***/
  ros::NodeHandle node_handle_;
  ros::Subscriber point_cloud_pos_sub_;
  ros::Subscriber point_cloud_neg_sub_;
  ros::Subscriber point_cloud_ukn_sub_;
  ros::Publisher  pose_array_pub_;
  ros::Publisher  marker_array_pub_;
  ros::Publisher  training_status_pub_;
  ros::Publisher prob_publisher_;
  /*** Parameters ***/
  bool print_fps_;
  std::string frame_id_;
  float cluster_tolerance_;
  int cluster_size_min_;
  int cluster_size_max_;
  bool training_finished;
  
  /*** SVM ***/
  std::vector<Feature> features_;
  struct svm_node *svm_node_;
  struct svm_model *svm_model_;
  struct svm_problem svm_problem_;
  struct svm_parameter svm_parameter_;
  float human_probability_;
  int svm_node_size_;
  
  /*** Online learning ***/
  int positive_;
  int negative_;
  int round_positives_;
  int round_negatives_;
  int max_trains_;
  int train_round_;
  std::vector<double> clusters_probability_;
  bool perform_learning;
  
public:
  Object3dDetector();
  ~Object3dDetector();
  
  void pointCloudPosCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  void pointCloudNegCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  void pointCloudUknCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2);
  
  /*** pcl::PointXYZHSV for radar data. xyz->xyz, hsv-> intensity, range, velocity (doppler) ***/
  void extractCluster(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, int type);
  void extractFeature(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, Feature &f);
  void saveFeature(Feature &f, struct svm_node *x);
  void classify(int type);
  void train();
  void loadFromFile();
};

Object3dDetector::Object3dDetector() {
  /*** Publishers ***/
  ros::NodeHandle private_nh("~");
  pose_array_pub_     = private_nh.advertise<geometry_msgs::PoseArray>("poses", 1);
  marker_array_pub_   = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 1);
  prob_publisher_   = private_nh.advertise<visualization_msgs::Marker>("markers_prob", 1);
  training_status_pub_ = private_nh.advertise<std_msgs::Bool>("training_status",1);
 
  /*** Parameters ***/
  private_nh.param<bool>("print_fps", print_fps_, false);
  private_nh.param<std::string>("frame_id", frame_id_, "base_radar_link");
  private_nh.param<float>("cluster_tolerance", cluster_tolerance_, 0.3);
  private_nh.param<int>("cluster_size_min", cluster_size_min_, 10);
  private_nh.param<int>("cluster_size_max", cluster_size_max_, 300);
  private_nh.param<float>("human_probability", human_probability_, 0.5);
  private_nh.param<int>("round_positives", round_positives_, 12);
  private_nh.param<int>("round_negatives", round_negatives_, 12);
  private_nh.param<int>("max_trains", max_trains_, 3);
  private_nh.param<bool>("perform_learning", perform_learning, true);
  
  /*** SVM ***/
  svm_node_ = (struct svm_node *)malloc((FEATURE_SIZE+1)*sizeof(struct svm_node)); // 1 more size for end index (-1)
  
  svm_parameter_.svm_type = C_SVC; // default C_SVC
  svm_parameter_.kernel_type = RBF; // default RBF
  svm_parameter_.degree = 3; // default 3
  svm_parameter_.gamma = 0.02; // default 1.0/(float)FEATURE_SIZE
  svm_parameter_.coef0 = 0; // default 0
  svm_parameter_.cache_size = 256; // default 100
  svm_parameter_.eps = 0.001; // default 0.001
  svm_parameter_.C = 8; // default 1
  svm_parameter_.nr_weight = 0;
  svm_parameter_.weight_label = NULL;
  svm_parameter_.weight = NULL;
  svm_parameter_.nu = 0.5;
  svm_parameter_.p = 0.1;
  svm_parameter_.shrinking = 0;
  svm_parameter_.probability = 1;
  
  svm_node_size_ = (round_positives_+round_negatives_)*max_trains_;
  svm_problem_.l = 0;
  svm_problem_.y = (double *)malloc(svm_node_size_*sizeof(double));
  svm_problem_.x = (struct svm_node **)malloc(svm_node_size_*sizeof(struct svm_node *));
  for(int i = 0; i < svm_node_size_; i++)
    svm_problem_.x[i] = (struct svm_node *)malloc((FEATURE_SIZE + 1)*sizeof(struct svm_node));
  
  /*** online learning ***/
  train_round_ = 0;
  positive_ = 0;
  negative_ = 0;

  //perform_learning = false;
  /*** pre_trained model ***/
  if (perform_learning == false){
	  // svm_model_ = svm_load_model("pedestrian.model");
	  // if(svm_save_model("pedestrian.modela", svm_model_) == 0) std::cout << "A model has been generated here: ~/.ros/pedestrian.model" << std::endl;
	  // train_round_ = 1;
    std::cout << "adsasd sadasdasd" <<std::endl;
    // loadFromFile();
  }
  svm_parameter_.degree = 3; // default 3
  svm_parameter_.gamma = 0.02; // default 1.0/(float)FEATURE_SIZE
  svm_parameter_.coef0 = 0; // default 0
  svm_parameter_.cache_size = 256; // default 100
  svm_parameter_.eps = 0.001; // default 0.001
  svm_parameter_.C = 8; // default 1
  svm_parameter_.nr_weight = 0;
  svm_parameter_.weight_label = NULL;
  svm_parameter_.weight = NULL;
  svm_parameter_.nu = 0.5;
  svm_parameter_.p = 0.1;
  svm_parameter_.shrinking = 0;
  svm_parameter_.probability = 1;

  
  /*** Subscribers ***/
  point_cloud_pos_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("positive", 100, &Object3dDetector::pointCloudPosCallback, this);
  point_cloud_neg_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("negative", 100, &Object3dDetector::pointCloudNegCallback, this);
  point_cloud_ukn_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("unknown", 100, &Object3dDetector::pointCloudUknCallback, this);
 
}

Object3dDetector::~Object3dDetector() {
  svm_free_and_destroy_model(&svm_model_);
  free(svm_node_);
  svm_destroy_param(&svm_parameter_);
  free(svm_problem_.y);
  for(int i = 0; i < svm_node_size_; i++) 
    free(svm_problem_.x[i]);
  free(svm_problem_.x);
}

int frames; clock_t start_time; bool reset = true;//fps
void Object3dDetector::pointCloudPosCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps
  
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);
  
  extractCluster(pcl_pc, 1); // 1 means positive examples
  classify(1); // 1 means positive examples
  if (perform_learning) train();
  
  if(print_fps_)if(++frames>10){std::cout<<"[radar_detector_ol]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<", positive = "<<positive_<<", negative = "<<negative_<<std::endl;reset = true;}//fps
}

void Object3dDetector::pointCloudUknCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  if(print_fps_)if(reset){frames=0;start_time=clock();reset=false;}//fps
  
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);
  
  extractCluster(pcl_pc, 0); // 0 means unknown examples passed for classification
  classify(0); // 0 means unknown examples passed for classification
  fprintf(stdout,"Classifying unknown\n");
  
  if(print_fps_)if(++frames>10){std::cout<<"[radar_detector_ol]: fps = "<<float(frames)/(float(clock()-start_time)/CLOCKS_PER_SEC)<<", timestamp = "<<clock()/CLOCKS_PER_SEC<<", positive = "<<positive_<<", negative = "<<negative_<<std::endl;reset = true;}//fps
}

void Object3dDetector::pointCloudNegCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2) {
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZHSV>);
  pcl::fromROSMsg(*ros_pc2, *pcl_pc);
  
  extractCluster(pcl_pc, -1); // -1 means negative examples
}

void Object3dDetector::extractCluster(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, int type) {
  features_.clear();

  pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZHSV>);
  tree->setInputCloud(pc);
  
  std_msgs::Bool training_finished;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> ec;
  /******* TODO: rqt_reconfigure *******/
  ec.setClusterTolerance(cluster_tolerance_);
  ec.setMinClusterSize(cluster_size_min_);
  ec.setMaxClusterSize(cluster_size_max_);
  /******* TODO: rqt_reconfigure *******/
  ec.setSearchMethod(tree);
  ec.setInputCloud(pc);
  ec.extract(cluster_indices);
   
  for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZHSV>);
    for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cluster->points.push_back(pc->points[*pit]);
//	std::cout << "Indices ol " <<   it->indices.size() << std::endl; 
    }
    cluster->width = cluster->size();
    cluster->height = 1;
    cluster->is_dense = true;
    
    Feature f;
    extractFeature(cluster, f);
    features_.push_back(f);
    //std::cout<<  " Size from Ol extraction " << cluster->size() << std::endl;
    if(train_round_ < max_trains_) {
      	training_finished.data=false;
      if(type == 1 && positive_ < round_positives_) {
        saveFeature(f, svm_problem_.x[svm_problem_.l]);
        svm_problem_.y[svm_problem_.l++] = 1;
        ++positive_;
        std::cout << "positive: " << positive_ << std::endl;
      }
      if(type == -1 && negative_ < round_negatives_) {
        saveFeature(f, svm_problem_.x[svm_problem_.l]);
        svm_problem_.y[svm_problem_.l++] = -1;
        ++negative_;
        std::cout << "negative: " << negative_ << std::endl;
	}
    }
    if(train_round_ == max_trains_ ){
	training_finished.data = true;
    }
    training_status_pub_.publish(training_finished); 
  }
}

/* *** Feature Extraction ***
 * f1 (1d): the number of points included in a cluster.
 * f2 (1d): the minimum distance of the cluster to the sensor.
 * => f1 and f2 should be used in pairs, since f1 varies with f2 changes.
 * f3 (6d): 3D covariance matrix of the cluster.
 * f4 (6d): the normalized moment of inertia tensor.
 * => Since both f3 and f4 are symmetric, we only use 6 elements from each as features.
 * f5 (20d): Slice feature for the cluster.
 * f6 (27d): Intensity.
 */

void computeMomentOfInertiaTensorNormalized(pcl::PointCloud<pcl::PointXYZHSV> &pc, Eigen::Matrix3f &moment_3d) {
  moment_3d.setZero();
  for(size_t i = 0; i < pc.size(); i++) {
    moment_3d(0,0) += pc[i].y*pc[i].y+pc[i].z*pc[i].z;
    moment_3d(0,1) -= pc[i].x*pc[i].y;
    moment_3d(0,2) -= pc[i].x*pc[i].z;
    moment_3d(1,1) += pc[i].x*pc[i].x+pc[i].z*pc[i].z;
    moment_3d(1,2) -= pc[i].y*pc[i].z;
    moment_3d(2,2) += pc[i].x*pc[i].x+pc[i].y*pc[i].y;
  }
  moment_3d(1, 0) = moment_3d(0, 1);
  moment_3d(2, 0) = moment_3d(0, 2);
  moment_3d(2, 1) = moment_3d(1, 2);
}

/* Upper half, and the left and right lower halves of a pedestrian. */
void compute3ZoneCovarianceMatrix(pcl::PointCloud<pcl::PointXYZHSV>::Ptr plane, Eigen::Vector4f &mean, float *partial_covariance_2d) {
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr zone_decomposed[3];
  for(int i = 0; i < 3; i++)
    zone_decomposed[i].reset(new pcl::PointCloud<pcl::PointXYZHSV>);
  for(size_t i = 0; i < plane->size(); i++) {
    if(plane->points[i].z >= mean(2)) { // upper half
      zone_decomposed[0]->points.push_back(plane->points[i]);
    } else {
      if(plane->points[i].y >= mean(1)) // left lower half
	zone_decomposed[1]->points.push_back(plane->points[i]);
      else // right lower half
	zone_decomposed[2]->points.push_back(plane->points[i]);
    }
  }
  
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  for(int i = 0; i < 3; i++) {
    pcl::compute3DCentroid(*zone_decomposed[i], centroid);
    pcl::computeCovarianceMatrix(*zone_decomposed[i], centroid, covariance);
    partial_covariance_2d[i*3+0] = covariance(0,0);
    partial_covariance_2d[i*3+1] = covariance(0,1);
    partial_covariance_2d[i*3+2] = covariance(1,1);
  }
}

void computeSlice(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, int n, float *slice) {
  Eigen::Vector4f pc_min, pc_max;
  pcl::getMinMax3D(*pc, pc_min, pc_max);
  
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr blocks[n];
  float itv = (pc_max[2] - pc_min[2]) / n;
  if(itv > 0) {
    for(int i = 0; i < n; i++) {
      blocks[i].reset(new pcl::PointCloud<pcl::PointXYZHSV>);
    }
    for(unsigned int i = 0, j; i < pc->size(); i++) {
      j = std::min((n-1), (int)((pc->points[i].z - pc_min[2]) / itv));
      blocks[j]->points.push_back(pc->points[i]);
    }
    
    Eigen::Vector4f block_min, block_max;
    for(int i = 0; i < n; i++) {
      if(blocks[i]->size() > 2) {
	pcl::PCA<pcl::PointXYZHSV> pca;
	pcl::PointCloud<pcl::PointXYZHSV>::Ptr block_projected(new pcl::PointCloud<pcl::PointXYZHSV>);
	pca.setInputCloud(blocks[i]);
	pca.project(*blocks[i], *block_projected);
	pcl::getMinMax3D(*block_projected, block_min, block_max);
      } else if(blocks[i]->size() > 0) {
	pcl::getMinMax3D(*blocks[i], block_min, block_max);
      } else {
	block_min.setZero();
	block_max.setZero();
      }
      slice[i*2] = block_max[0] - block_min[0];
      slice[i*2+1] = block_max[1] - block_min[1];
    }
  } else {
    for(int i = 0; i < 20; i++) {
      slice[i] = 0;
    }
  }
}

void computeIntensity(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, int bins, float *intensity) {
  float sum = 0, mean = 0, sum_dev = 0;
  float min = FLT_MAX, max = -FLT_MAX;
  for(int i = 0; i < 27; i++)
    intensity[i] = 0;
  
  for(size_t i = 0; i < pc->size(); i++) {
    sum += pc->points[i].h;
    min = std::min(min, pc->points[i].h);
    max = std::max(max, pc->points[i].h);
  }
  mean = sum / pc->size();
  
  for(size_t i = 0; i < pc->size(); i++) {
    sum_dev += (pc->points[i].h-mean)*(pc->points[i].h-mean);
    int ii = std::min(float(bins-1), std::floor((pc->points[i].h-min)/((max-min)/bins)));
    intensity[ii]++;
  }
  intensity[25] = sqrt(sum_dev/pc->size());
  intensity[26] = mean;
}

void Object3dDetector::extractFeature(pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc, Feature &f) {
  Eigen::Vector4f centroid, min, max;
  pcl::compute3DCentroid(*pc, centroid);
  pcl::getMinMax3D(*pc, min, max);
  
  f.centroid = centroid;
  f.min = min;
  f.max = max;

  // f1: Number of points included the cluster.
  f.number_points = pc->size();
  // f2: The minimum distance to the cluster.
  f.min_distance = FLT_MAX;
  float d2; //squared Euclidean distance
  for(int i = 0; i < pc->size(); i++) {
    d2 = pc->points[i].x*pc->points[i].x + pc->points[i].y*pc->points[i].y + pc->points[i].z*pc->points[i].z;
    if(f.min_distance > d2)
      f.min_distance = d2;
  }
  
  pcl::PCA<pcl::PointXYZHSV> pca;
  pcl::PointCloud<pcl::PointXYZHSV>::Ptr pc_projected(new pcl::PointCloud<pcl::PointXYZHSV>);
  pca.setInputCloud(pc);
  pca.project(*pc, *pc_projected);
  // f3: 3D covariance matrix of the cluster.
  pcl::computeCovarianceMatrixNormalized(*pc_projected, centroid, f.covariance_3d);
  // f4: The normalized moment of inertia tensor.
  computeMomentOfInertiaTensorNormalized(*pc_projected, f.moment_3d);
  // f5: Slice feature for the cluster.
  computeSlice(pc, 10, f.slice);
  // f6: Intensity.
  computeIntensity(pc, 25, f.intensity);
  // f7: Average velocity.
  float sum = 0;
  for(int i = 0; i < pc->size(); i++) {
    sum += pc->points[i].v;
  }
  f.velocity = sum / pc->size();
}

void Object3dDetector::saveFeature(Feature &f, struct svm_node *x) {
  x[0].index  = 1;  x[0].value  = f.number_points; // libsvm indices start at 1
  x[1].index  = 2;  x[1].value  = f.min_distance;
  x[2].index  = 3;  x[2].value  = f.covariance_3d(0,0);
  x[3].index  = 4;  x[3].value  = f.covariance_3d(0,1);
  x[4].index  = 5;  x[4].value  = f.covariance_3d(0,2);
  x[5].index  = 6;  x[5].value  = f.covariance_3d(1,1);
  x[6].index  = 7;  x[6].value  = f.covariance_3d(1,2);
  x[7].index  = 8;  x[7].value  = f.covariance_3d(2,2);
  x[8].index  = 9;  x[8].value  = f.moment_3d(0,0);
  x[9].index  = 10; x[9].value  = f.moment_3d(0,1);
  x[10].index = 11; x[10].value = f.moment_3d(0,2);
  x[11].index = 12; x[11].value = f.moment_3d(1,1);
  x[12].index = 13; x[12].value = f.moment_3d(1,2);
  x[13].index = 14; x[13].value = f.moment_3d(2,2);
  for(int i = 0; i < 20; i++) {
    x[i+14].index = i+15;
    x[i+14].value = f.slice[i];
  }
  for(int i = 0; i < 27; i++) {
    x[i+34].index = i+35;
    x[i+34].value = f.intensity[i];
  }
  x[61].index = 62; x[61].value = f.velocity;
  x[FEATURE_SIZE].index = -1;
  
  /****** Debug print ******/
  // for(int i = 0; i < FEATURE_SIZE; i++) {
  //   std::cout << x[i].index << ":" << x[i].value << " ";
  //   std::cout << std::endl;
  // }
  /****** Debug print ******/
}

void Object3dDetector::classify(int type) {
  geometry_msgs::PoseArray pose_array;
  visualization_msgs::MarkerArray marker_array;
  int find_type = 1;
  visualization_msgs::Marker marker_prob;
  for(std::vector<Feature>::iterator it = features_.begin(); it != features_.end(); it++) {
	  bool svm_find_human = false;

	  if(type == 1)
	  {
		  svm_find_human = true;
      find_type = 0;
    }
    else{
		  if(train_round_ > 0) {
			  /*** scale data ***/
			  saveFeature(*it, svm_node_);
			  for(int i = 0; i < FEATURE_SIZE; i++) {
				  if(svm_range_[i][0] == svm_range_[i][1]) {
					  continue;
				  }
				  if(svm_node_[i].value == svm_range_[i][0]) {
					  svm_node_[i].value = svm_xlower_;
				  } else if(svm_node_[i].value == svm_range_[i][1]) {
					  svm_node_[i].value = svm_xupper_;
				  } else {
					  svm_node_[i].value = svm_xlower_ + (svm_xupper_ - svm_xlower_) * (svm_node_[i].value - svm_range_[i][0]) / (svm_range_[i][1] - svm_range_[i][0]);
				  }
			  }

			  /*** predict ***/
			  if(svm_check_probability_model(svm_model_)) {
				  double prob_estimates[svm_model_->nr_class];
				  svm_predict_probability(svm_model_, svm_node_, prob_estimates);
				  clusters_probability_.push_back(prob_estimates[0]);
          geometry_msgs::Point point;
          point.x = it->centroid[0];
          point.y = it->centroid[1];
          point.z = prob_estimates[0];
          marker_prob.points.push_back(point);
				  if(prob_estimates[0] > human_probability_) {
					  svm_find_human = true;
            find_type = 0;
				  }
			  } else {
				  if(svm_predict(svm_model_, svm_node_) == 1)
					  svm_find_human = true;
            find_type = 0;
        }
      }
	  }
    marker_prob.scale.x = 0.1;
    marker_prob.scale.y = 0.1;
    marker_prob.scale.z = 0.1;
    marker_prob.color.a = 1.0;
    marker_prob.color.r = 0.0;
    marker_prob.color.g = 0.0;
    marker_prob.color.b = 1.0;
	  marker_prob.header.stamp = ros::Time::now();
	  marker_prob.header.frame_id = "map";
	  marker_prob.id = 7;
    marker_prob.type = 7;

    prob_publisher_.publish(marker_prob);

	  /*** cluster pose ***/
	  geometry_msgs::Pose pose;
	  pose.position.x = it->centroid[0];
	  pose.position.y = it->centroid[1];
	  pose.position.z = it->centroid[2];
	  pose.orientation.w = 1;

	  /*** bounding box ***/
	  visualization_msgs::Marker marker;
	  marker.header.stamp = ros::Time::now();
	  marker.header.frame_id = frame_id_;
	  marker.ns = "radar_detector_ol";
	  marker.id = it-features_.begin();
	  marker.type = visualization_msgs::Marker::LINE_LIST;
	  geometry_msgs::Point p[24];
	  p[0].x = it->max[0]; p[0].y = it->max[1]; p[0].z = it->max[2];
	  p[1].x = it->min[0]; p[1].y = it->max[1]; p[1].z = it->max[2];
	  p[2].x = it->max[0]; p[2].y = it->max[1]; p[2].z = it->max[2];
	  p[3].x = it->max[0]; p[3].y = it->min[1]; p[3].z = it->max[2];
	  p[4].x = it->max[0]; p[4].y = it->max[1]; p[4].z = it->max[2];
	  p[5].x = it->max[0]; p[5].y = it->max[1]; p[5].z = it->min[2];
	  p[6].x = it->min[0]; p[6].y = it->min[1]; p[6].z = it->min[2];
	  p[7].x = it->max[0]; p[7].y = it->min[1]; p[7].z = it->min[2];
	  p[8].x = it->min[0]; p[8].y = it->min[1]; p[8].z = it->min[2];
	  p[9].x = it->min[0]; p[9].y = it->max[1]; p[9].z = it->min[2];
	  p[10].x = it->min[0]; p[10].y = it->min[1]; p[10].z = it->min[2];
	  p[11].x = it->min[0]; p[11].y = it->min[1]; p[11].z = it->max[2];
	  p[12].x = it->min[0]; p[12].y = it->max[1]; p[12].z = it->max[2];
	  p[13].x = it->min[0]; p[13].y = it->max[1]; p[13].z = it->min[2];
	  p[14].x = it->min[0]; p[14].y = it->max[1]; p[14].z = it->max[2];
	  p[15].x = it->min[0]; p[15].y = it->min[1]; p[15].z = it->max[2];
	  p[16].x = it->max[0]; p[16].y = it->min[1]; p[16].z = it->max[2];
	  p[17].x = it->max[0]; p[17].y = it->min[1]; p[17].z = it->min[2];
	  p[18].x = it->max[0]; p[18].y = it->min[1]; p[18].z = it->max[2];
	  p[19].x = it->min[0]; p[19].y = it->min[1]; p[19].z = it->max[2];
	  p[20].x = it->max[0]; p[20].y = it->max[1]; p[20].z = it->min[2];
	  p[21].x = it->min[0]; p[21].y = it->max[1]; p[21].z = it->min[2];
	  p[22].x = it->max[0]; p[22].y = it->max[1]; p[22].z = it->min[2];
	  p[23].x = it->max[0]; p[23].y = it->min[1]; p[23].z = it->min[2];
	  for(int i = 0; i < 24; i++) {
		  marker.points.push_back(p[i]);
	  }
	  marker.scale.x = 0.02;
	  marker.color.a = 1.0;
	  if(svm_find_human || true) {
		  marker.color.r = find_type;
		  marker.color.g = 1.0;
		  marker.color.b = 0.5;
		  marker.lifetime = ros::Duration(0.1);
		  marker_array.markers.push_back(marker);
	  }
    if(svm_find_human) {
      pose_array.poses.push_back(pose);
	  }
  }
  
  /*** publish pose and marker ***/
  if(pose_array.poses.size()) {
    pose_array.header.stamp = ros::Time::now();
    pose_array.header.frame_id = frame_id_;
    pose_array_pub_.publish(pose_array);
  }
  
  if(marker_array.markers.size()) {
    marker_array_pub_.publish(marker_array);
  }
}

void Object3dDetector::train() {

	printf("%i %i %i %i\n", positive_, negative_, round_positives_, round_negatives_);
  if((positive_+negative_) < (round_positives_+round_negatives_))
  {
	  printf("Not enough data for training, aborting\n");
	  return;
  }
  
  clock_t t = clock();
  std::cout << "\n****** Training round " << (train_round_+1) << " started ******\n" << std::endl;
  
  /*** scale back the previous data ***/
  for(int i = 0; i < (round_positives_+round_negatives_)*train_round_; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      if(svm_range_[j][0] == svm_range_[j][1]) // skip single-valued attribute
	continue;
      if(svm_problem_.x[i][j].value == svm_xlower_)
	svm_problem_.x[i][j].value = svm_range_[j][0];
      else if(svm_problem_.x[i][j].value == svm_xupper_)
	svm_problem_.x[i][j].value = svm_range_[j][1];
      else
	svm_problem_.x[i][j].value = svm_range_[j][0] + (svm_problem_.x[i][j].value - svm_xlower_) * (svm_range_[j][1] - svm_range_[j][0]) / (svm_xupper_ - svm_xlower_);
    }
  }
  
  /*** save data to file ***/
  std::ofstream s;
  s.open("svm_training_data");
  for(int i = 0; i < svm_problem_.l; i++) {
    s << svm_problem_.y[i];
    for(int j = 0; j < FEATURE_SIZE; j++)
      s << " " << svm_problem_.x[i][j].index << ":" << svm_problem_.x[i][j].value;
    s << "\n";
  }
  s.close();
  
  /*** scale the current data ***/
  for(int i = 0; i < FEATURE_SIZE; i++) {
    svm_range_[i][0] = FLT_MAX; // min range
    svm_range_[i][1] = -FLT_MAX; // max range
  }
  for(int i = 0; i < svm_problem_.l; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      svm_range_[j][0] = std::min(svm_range_[j][0], (float)svm_problem_.x[i][j].value);
      svm_range_[j][1] = std::max(svm_range_[j][1], (float)svm_problem_.x[i][j].value);
    }
  }
  
  for(int i = 0; i < svm_problem_.l; i++) {
    for(int j = 0; j < FEATURE_SIZE; j++) {
      if(svm_range_[j][0] == svm_range_[j][1]) // skip single-valued attribute
	continue;
      if(svm_problem_.x[i][j].value == svm_range_[j][0])
	svm_problem_.x[i][j].value = svm_xlower_;
      else if(svm_problem_.x[i][j].value == svm_range_[j][1])
	svm_problem_.x[i][j].value = svm_xupper_;
      else
	svm_problem_.x[i][j].value = svm_xlower_ + (svm_xupper_ - svm_xlower_) * (svm_problem_.x[i][j].value - svm_range_[j][0]) / (svm_range_[j][1] - svm_range_[j][0]);
    }
  }
  
  /*** train ***/
    s.open("svm_training_data");
    for(int i = 0; i < svm_problem_.l; i++) {
      s << svm_problem_.y[i];
      for(int j = 0; j < FEATURE_SIZE; j++)
	s << " " << svm_problem_.x[i][j].index << ":" <<  svm_problem_.x[i][j].value;
      s << "\n";
    }
    s.close();
    
    std::cout << "Finding the best training parameters ..." << std::endl;
    if(svm_check_parameter(&svm_problem_, &svm_parameter_) == NULL) {
      char result[100];
      FILE *fp = popen("./grid.py svm_training_data", "r");
      if(fp == NULL) {
	std::cout << "Can not run cross validation!" << std::endl;
      } else {
	if(fgets(result, 100, fp) != NULL) {
	  char *pch = strtok(result, " ");
	  svm_parameter_.C = atof(pch); pch = strtok(NULL, " ");
	  svm_parameter_.gamma = atof(pch); pch = strtok(NULL, " ");
	  float rate = atof(pch);
	  std::cout << "Best c=" << svm_parameter_.C << ", g=" << svm_parameter_.gamma << " CV rate=" << rate << std::endl;
	}
      }
      pclose(fp);
    }
  svm_model_ = svm_train(&svm_problem_, &svm_parameter_);

  /*** reset parameters ***/
  if(train_round_ < max_trains_) {
    train_round_++;
    positive_ = 0;
    negative_ = 0;
  }
  clusters_probability_.clear();
  
  if(svm_save_model("pedestrian.model", svm_model_) == 0) std::cout << "A model has been generated here: ~/.ros/pedestrian.model" << std::endl;
  /*** debug saving ***/
  std::cout << "\n****** Training round " << train_round_ << " finished with " << float(clock()-t)/CLOCKS_PER_SEC << " seconds ******\n" << std::endl;
}

void Object3dDetector::loadFromFile()
{
	printf("here\n");
	std::ifstream input("svm_training_data");
	int ctr = 0;
	for(std::string line; getline(input, line);)
	{
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(" "));

		std::istringstream is(line);

		is >> svm_problem_.y[ctr];

		if(svm_problem_.y[ctr] == 1)

			positive_++;
		else
			negative_++;

		for(int i = 0; i < 62; i++)
		{
			is >> svm_problem_.x[ctr][i].value;
			svm_problem_.x[ctr][i].index = i;
		}

		ctr++;
	}
	svm_problem_.l = ctr;
	train();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "radar_detector_ol");
  Object3dDetector d;
  ros::spin();
  return 0;
}
