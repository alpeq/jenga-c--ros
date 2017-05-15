#ifndef USC_H
#define USC_H

#include <pcl/features/usc.h>

#include "common.h"

#define USC_RADIUS		0.05


/***** This file contains the following functions *****/
Eigen::Matrix4f USC(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);
pcl::PointCloud<pcl::UniqueShapeContext1960> ExtractUSC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float USCRadius = -1);
void nearest_feature(const pcl::UniqueShapeContext1960 &query, const pcl::PointCloud<pcl::UniqueShapeContext1960> &target, int &idx, float &distsq);
inline float dist_sq(const pcl::UniqueShapeContext1960 &query, const pcl::UniqueShapeContext1960 &target);


Eigen::Matrix4f USC(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene) {
	//Create the histograms to contain the SHOT
	pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr modelUSC(new pcl::PointCloud<pcl::UniqueShapeContext1960>());	//Create a cloud to save results
	pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr sceneUSC(new pcl::PointCloud<pcl::UniqueShapeContext1960>());	//Create a cloud to save results

	//Extract the SHOT
	*modelUSC = ExtractUSC(model);
	*sceneUSC = ExtractUSC(scene);

	//Match the features
	pcl::Correspondences corr(modelUSC->size());
	for(size_t i = 0; i < modelUSC->size(); ++i) {
		corr[i].index_query = i;
		nearest_feature(modelUSC->points[i], *sceneUSC, corr[i].index_match, corr[i].distance);
	}

	//Estimate the pose using RANSAC
	Eigen::Matrix4f pose = RANSAC(model, scene, corr);

	return pose;

}


pcl::PointCloud<pcl::UniqueShapeContext1960> ExtractUSC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float USCRadius) {
	//If no FPFP radius is specified, use the standard value in the top of the document
	if (USCRadius == -1)
		USCRadius = USC_RADIUS;


	//Create USC estimator
	pcl::UniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc_est;

        pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr descriptors(new pcl::PointCloud<pcl::UniqueShapeContext1960>());
	//Set the attributes
        usc_est.setInputCloud(cloud);
        usc_est.setRadiusSearch(USCRadius);
        usc_est.setMinimalRadius(USCRadius / 10.0);
        usc_est.setPointDensityRadius(USCRadius/ 5.0);
        usc_est.setLocalRadius(USCRadius);
	//Compute the SHOT signatures
        usc_est.compute(*descriptors);


	return *descriptors;
}


inline float dist_sq(const pcl::UniqueShapeContext1960 &query, const pcl::UniqueShapeContext1960 &target) {
    float result = 0.0;
    for(int i = 0; i < pcl::UniqueShapeContext1960::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }
    
    return result;
}


void nearest_feature(const pcl::UniqueShapeContext1960 &query, const pcl::PointCloud<pcl::UniqueShapeContext1960> &target, int &idx, float &distsq) {
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
