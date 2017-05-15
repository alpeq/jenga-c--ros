#ifndef ICP_H
#define ICP_H

#include <pcl/registration/icp.h>

/***** This file contains the following functions *****/
Eigen::Matrix4f ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

Eigen::Matrix4f ICP(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene) {
	//Create ICP object
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(model);
	icp.setInputTarget(scene);

	//Run ICP
	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*result);

	return icp.getFinalTransformation();

}

#endif
