#ifndef COMMON_H
#define COMMON_H

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/random.h>
#include <pcl/filters/voxel_grid.h>

#define NORM_RADIUS		0.03
#define RANSAC_ITR		500
#define INLIER_TRSH		0.001

/***** This file contains the following functions *****/
pcl::PointCloud<pcl::Normal> computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float normRadius = -1);
Eigen::Matrix4f RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::Correspondences corr, int ransacItr = -1, int inlierTrsh = -1);
void DownSampler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

pcl::PointCloud<pcl::Normal> computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float normRadius) {
	//If no raduis is specified for calculating the normals, use the standard value defined in the beginning of this file 
	if (normRadius == -1)
		normRadius = NORM_RADIUS;

	//Create the normal estimator and the kdtree used as a search method
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());

	//Set the attributes
	normalEstimator.setInputCloud(cloud);
	normalEstimator.setRadiusSearch(normRadius);
	normalEstimator.setSearchMethod(kdtree);

	//Compute the cloud normals
	pcl::PointCloud<pcl::Normal> cloudNormals;
	normalEstimator.compute(cloudNormals);		

	return cloudNormals;
}


Eigen::Matrix4f RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::Correspondences corr, int ransacItr, int inlierTrsh) {
	//If number of iterations and inlier treshold not given, use the standard values in the beginning of this file
	if (ransacItr == -1)
		ransacItr = RANSAC_ITR;
	if (inlierTrsh == -1)
		inlierTrsh = INLIER_TRSH;
	//Prepare for RANSAC
    	pcl::search::KdTree<pcl::PointXYZ> scene_tree;			//Create a k-d tree for scene
    	scene_tree.setInputCloud(scene);
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();		//Create Matrix to save the pose
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned(new pcl::PointCloud<pcl::PointXYZ>);		//Create a new point cloud for the aligned result
	float penalty = FLT_MAX;

	// Start RANSAC
	pcl::common::UniformGenerator<int> gen(0, corr.size() - 1);	//UniformGenerator produces random value

	for(size_t i = 0; i < ransacItr; ++i) {
		if((i + 1) % 100 == 0)
			cout << "\t" << "Iteration " << i+1 << " of " << ransacItr << endl;

		// Sample 3 random correspondences
		std::vector<int> model_id(3);
		std::vector<int> scene_id(3);
		for(int j = 0; j < 3; ++j) {
			const int idx = gen.run();
			model_id[j] = corr[idx].index_query;
			scene_id[j] = corr[idx].index_match;
		}

		// Estimate transformation
		Eigen::Matrix4f transformation_matrix;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_est;
		transformation_est.estimateRigidTransformation(*model, model_id, *scene, scene_id, transformation_matrix);
            
		// Apply pose
		transformPointCloud(*model, *model_aligned, transformation_matrix);

		// Validate
		std::vector<std::vector<int> > idx;
		std::vector<std::vector<float> > distsq;
		scene_tree.nearestKSearch(*model_aligned, std::vector<int>(), 1, idx, distsq);
            
		// Compute inliers and RMSE
		size_t inliers = 0;
		float rmse = 0;
		for(size_t j = 0; j < distsq.size(); ++j)
			if(distsq[j][0] <= INLIER_TRSH)
				++inliers, rmse += distsq[j][0];
		rmse = sqrtf(rmse / inliers);

		// Evaluate a penalty function
		const float outlier_rate = 1.0f - float(inliers) / model->size();
		//const float penaltyi = rmse;
		const float penaltyi = outlier_rate;

		// Update result
		if(penaltyi < penalty) {
			cout << "\t" << "New pose found with " << inliers << " inliers." << endl;
			penalty = penaltyi;
			pose = transformation_matrix;
		}
	}

	return pose;

}

void DownSampler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  //Reduce the number of points in the cloud to increase the computational speed of the following algorithms
  pcl::VoxelGrid<pcl::PointXYZ> downSampler;            //Create a downsampler object
  downSampler.setInputCloud (cloud_in);
  downSampler.setLeafSize (0.19f, 0.19f, 0.19f);
  downSampler.filter(*cloud_out);

}

#endif
