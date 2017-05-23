#ifndef FPFH_H
#define FPFH_H

#include <pcl/features/fpfh.h>

#include "common.h"

#define FPFH_RADIUS		FEATURE_RADIUS;


/***** This file contains the following functions *****/
Eigen::Matrix4f FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);
pcl::PointCloud<pcl::FPFHSignature33> ExtractFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float fpfhRadius = -1);
void nearest_feature(const pcl::FPFHSignature33 &query, const pcl::PointCloud<pcl::FPFHSignature33> &target, int &idx, float &distsq);
inline float dist_sq(const pcl::FPFHSignature33 &query, const pcl::FPFHSignature33 &target);


Eigen::Matrix4f FPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene) {
	//take time
		high_resolution_clock::time_point t1 = high_resolution_clock::now();

	//Create the histograms to contain the FPFH
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr modelFPFH(new pcl::PointCloud<pcl::FPFHSignature33>());	//Create a cloud to save results
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr sceneFPFH(new pcl::PointCloud<pcl::FPFHSignature33>());	//Create a cloud to save results

	//Extract the FPFH
	*modelFPFH = ExtractFPFH(model);
	*sceneFPFH = ExtractFPFH(scene);

	//Match the features
	pcl::Correspondences corr(modelFPFH->size());
	for(size_t i = 0; i < modelFPFH->size(); ++i) {
		corr[i].index_query = i;
		nearest_feature(modelFPFH->points[i], *sceneFPFH, corr[i].index_match, corr[i].distance);
	}

 Present_and_Report(t1,model, scene, corr);
	//Estimate the pose using RANSAC
	Eigen::Matrix4f pose = RANSAC(model, scene, corr);

	return pose;

}


pcl::PointCloud<pcl::FPFHSignature33> ExtractFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float fpfhRadius) {
	//If no FPFP radius is specified, use the standard value in the top of the document
	if (fpfhRadius == -1)
		fpfhRadius = FEATURE_RADIUS;

	//Calculate cloud normals
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	*cloudNormals = computeNormals(cloud);

	//Create FPFH estimator and kd-tree
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> FPFHEstimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	//Set the attributes
	FPFHEstimator.setInputCloud(cloud);
	FPFHEstimator.setInputNormals(cloudNormals);
	FPFHEstimator.setSearchMethod(kdtree);
	FPFHEstimator.setRadiusSearch(fpfhRadius);

	//Compute the FPFH signatures
	pcl::PointCloud<pcl::FPFHSignature33> cloudFPFH;
	FPFHEstimator.compute(cloudFPFH);

	return cloudFPFH;
}


inline float dist_sq(const pcl::FPFHSignature33 &query, const pcl::FPFHSignature33 &target) {
    float result = 0.0;
    for(int i = 0; i < pcl::FPFHSignature33::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}


void nearest_feature(const pcl::FPFHSignature33 &query, const pcl::PointCloud<pcl::FPFHSignature33> &target, int &idx, float &distsq) {
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
