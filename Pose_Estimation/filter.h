#ifndef FILTER_H
#define FILTER_H

//Compute resolution (not implemented): http://pointclouds.org/documentation/tutorials/correspondence_grouping.php

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/segmentation/sac_segmentation.h>

#define FILTER_AXIS     "z"
#define FILTER_MIN      0
#define FILTER_MAX      1.1
#define INLIER_TRESH    0.01

/***** This file contains the following functions *****/
void FilterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
void FilterTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);

void FilterBackground(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    //Create filter object
    pcl::PassThrough<pcl::PointXYZ> filter;

    //Remove the points further away than FILTER_MAX meters in the FILTER_AXIS direction
    filter.setInputCloud(cloud_in);
    filter.setFilterFieldName(FILTER_AXIS);
    filter.setFilterLimits(FILTER_MIN, FILTER_MAX);
    filter.filter(*cloud_out);
}

void FilterTable(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
    //Find the table plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  //Created to store plane coeff.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);                 //Created to store plane inlier indices
    pcl::SACSegmentation<pcl::PointXYZ> seg;                                //Created to detect table plane
    seg.setModelType (pcl::SACMODEL_PLANE);                                 //Specify model
    seg.setMethodType (pcl::SAC_RANSAC);                                    //Specify RANSAC as method
    seg.setDistanceThreshold (INLIER_TRESH);                                //Inlier treshold
    seg.setInputCloud (cloud_in);
    seg.segment (*inliers, *coefficients);

    //Remove the inliers
    for (int i = inliers->indices.size()-1; i > 0; --i)
        cloud_out->erase(cloud_out->begin()+inliers->indices[i]);
}

#endif