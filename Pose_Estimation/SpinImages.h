#ifndef SPINIMAGES_H
#define SPINIMAGES_H

#include <pcl/point_types.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/correspondence.h>

#include "common.h"

//#define SPIN_RADIUS			0.1 //not really necessary?
//int BIN_SIZE = 0;




/***** This file contains the following functions *****/
Eigen::Matrix4f SpinImages(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);
pcl::PointCloud<pcl::Histogram<153> > ExtractSpinImages(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float spinRadius = -1);
inline float dist_sq(const pcl::Histogram<153> &query, const pcl::Histogram<153> &target);
void nearest_feature(const pcl::Histogram<153> &query, const pcl::PointCloud<pcl::Histogram<153> > &target, int &idx, float &distsq);

int spinMatches =0;
//spin images:
Eigen::Matrix4f SpinImages(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene) {

//take time
	high_resolution_clock::time_point t1 = high_resolution_clock::now();


		//Create the histograms to contain the images
	pcl::PointCloud<pcl::Histogram<153> >::Ptr modelImages(new pcl::PointCloud<pcl::Histogram<153> >());
	pcl::PointCloud<pcl::Histogram<153> >::Ptr sceneImages(new pcl::PointCloud<pcl::Histogram<153> >());

	//Compute the spin images
	*modelImages = ExtractSpinImages(model);
	*sceneImages = ExtractSpinImages(scene);

	//Match the features
	cout << "Done!\n";
	cout  << "Matching features...                                          ";
  cout.flush();

	pcl::Correspondences corr(modelImages->size());
	for(size_t i = 0; i < modelImages->size(); ++i) {
		corr[i].index_query = i;
		nearest_feature(modelImages->points[i], *sceneImages, corr[i].index_match, corr[i].distance);
		spinMatches= i;
	}

 Present_and_Report(t1,model, scene, corr, spinMatches);

	//Estimate the pose using RANSAC
	Eigen::Matrix4f pose = RANSAC(model, scene, corr);
	return pose;
}


pcl::PointCloud<pcl::Histogram<153> > ExtractSpinImages(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float spinRadius) {
	//If spinRadius not specified, use the SPIN_RADIUS value, otherwise use the given value
	if (spinRadius == -1)
		spinRadius = FEATURE_RADIUS;

	//Prepare the ImageSpinner
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > image_spinner;
	pcl::PointCloud<pcl::Histogram<153> > cloudSpinImages;

	//Compute the cloud normals
	pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
	*cloudNormals = computeNormals(cloud);

	//Set attributes
	image_spinner.setRadiusSearch(spinRadius);
	image_spinner.setInputCloud(cloud);
	image_spinner.setInputNormals(cloudNormals);

	//Compute the spin images
	image_spinner.compute(cloudSpinImages);
	return cloudSpinImages;
}

inline float dist_sq(const pcl::Histogram<153> &query, const pcl::Histogram<153> &target) {
    float result = 0.0;
    for(int i = 0; i < pcl::Histogram<153>::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += diff * diff;
    }

    return result;
}

/*
cout <<"Done!"<< '\n';
clog << "Normals, extraction, and matching done in time: \n";
cout.flush();
*/

void nearest_feature(const pcl::Histogram<153> &query, const pcl::PointCloud<pcl::Histogram<153> > &target, int &idx, float &distsq) {
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
