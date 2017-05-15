#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "SpinImages.h"
#include "FPFH.h"
#include "SHOT.h"
#include "USC.h"
#include "ICP.h"
#include "common.h"

#define SCENE_PATH				"scene.pcd"
#define MODEL_PATH				"object-global.pcd"


int main (int argc, char* argv[]) {
    //Read the input
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(MODEL_PATH, *model) == -1 || pcl::io::loadPCDFile<pcl::PointXYZ>(SCENE_PATH, *scene) == -1) {
        cout << "Error: Couldn't read file." << endl;
        return -1;
    }

	//Display initial state
    pcl::visualization::PCLVisualizer init_view("Initial view");
    init_view.addPointCloud<pcl::PointXYZ>(scene, "Scene");
    init_view.addPointCloud<pcl::PointXYZ>(model, "Model");
	init_view.spin();

	//Do the pose estimation using SpinImages or FPFH
	Eigen::Matrix4f pose = USC(model, scene);
//	Eigen::Matrix4f pose = SpinImages(model, scene);
//	Eigen::Matrix4f pose = SHOT(model, scene);
//	Eigen::Matrix4f pose = FPFH(model, scene);

	// Apply the pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedModel (new pcl::PointCloud<pcl::PointXYZ>);
	transformPointCloud(*model, *alignedModel, pose);

	//View result
	pcl::visualization::PCLVisualizer preICP_viewer("preICP");
	preICP_viewer.addPointCloud<pcl::PointXYZ>(alignedModel, "Model aligned");
	preICP_viewer.addPointCloud<pcl::PointXYZ>(scene, "Scene");
	preICP_viewer.spin();

	//Run ICP and save the Transformation matrix into pose
	pose = ICP(alignedModel, scene);

	// Apply the pose found by ICP
	transformPointCloud(*alignedModel, *alignedModel, pose);

	//View result
	pcl::visualization::PCLVisualizer ICP_viewer("Final");
	ICP_viewer.addPointCloud<pcl::PointXYZ>(alignedModel, "Model aligned");
	ICP_viewer.addPointCloud<pcl::PointXYZ>(scene, "Scene");
	ICP_viewer.spin();

	//Save output
//	pcl::io::savePCDFileASCII("output.pcd", *alignedModel);


}
