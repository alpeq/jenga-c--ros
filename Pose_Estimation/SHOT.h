#ifndef SHOT_H
#define SHOT_H

#include <pcl/features/shot.h>

#include "common.h"

#define SHOT_RADIUS		0.1


/***** This file contains the following functions *****/
Eigen::Matrix4f SHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);
pcl::PointCloud<pcl::SHOT352> ExtractSHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float SHOTRadius = -1);
void nearest_feature(const pcl::SHOT352 &query, const pcl::PointCloud<pcl::SHOT352> &target, int &idx, float &distsq);
inline float dist_sq(const pcl::SHOT352 &query, const pcl::SHOT352 &target);


Eigen::Matrix4f SHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene) {
	//Create the histograms to contain the SHOT
	pcl::PointCloud<pcl::SHOT352>::Ptr modelSHOT(new pcl::PointCloud<pcl::SHOT352>());	//Create a cloud to save results
	pcl::PointCloud<pcl::SHOT352>::Ptr sceneSHOT(new pcl::PointCloud<pcl::SHOT352>());	//Create a cloud to save results

	//Extract the SHOT
	*modelSHOT = ExtractSHOT(model);
	*sceneSHOT = ExtractSHOT(scene);

	//Match the features
	pcl::Correspondences corr(modelSHOT->size());
	for(size_t i = 0; i < modelSHOT->size(); ++i) {
		corr[i].index_query = i;
		nearest_feature(modelSHOT->points[i], *sceneSHOT, corr[i].index_match, corr[i].distance);
	}

	//Estimate the pose using RANSAC
	Eigen::Matrix4f pose = RANSAC(model, scene, corr);

	return pose;

}


pcl::PointCloud<pcl::SHOT352> ExtractSHOT(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float SHOTRadius) {
	//If no FPFP radius is specified, use the standard value in the top of the document
	if (SHOTRadius == -1)
		SHOTRadius = SHOT_RADIUS;

	//Calculate cloud normals
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	*cloudNormals = computeNormals(cloud);

	//Create SHOT estimator and kd-tree
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> SHOTEstimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	//Set the attributes
	SHOTEstimator.setInputCloud(cloud);
	SHOTEstimator.setInputNormals(cloudNormals);
	SHOTEstimator.setSearchMethod(kdtree);
	SHOTEstimator.setRadiusSearch(SHOTRadius);

	//Compute the SHOT signatures
	pcl::PointCloud<pcl::SHOT352> cloudSHOT;
	SHOTEstimator.compute(cloudSHOT);

	return cloudSHOT;
}


inline float dist_sq(const pcl::SHOT352 &query, const pcl::SHOT352 &target) {
    float result = 0.0;
    for(int i = 0; i < pcl::SHOT352::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}


void nearest_feature(const pcl::SHOT352 &query, const pcl::PointCloud<pcl::SHOT352> &target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti < distsq) {
            idx = i;
            distsq = disti;
        }
    }
}

#endif
