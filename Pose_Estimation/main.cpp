#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <chrono>

#include "SpinImages.h"
#include "FPFH.h"
#include "SHOT.h"
//#include "USC.h"
#include "ICP.h"
//#include "filter.h"
#include "common.h"

using namespace std::chrono;

//placeholder definition of select feature function
Eigen::Matrix4f select_feature_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene);

int main (int argc, char* argv[]) {

clog.rdbuf(ofs.rdbuf()); //Redirecting the clog buffer stream, to file

    cout << "Loading the input files...                                ";
    cout.flush();

    Load_Settings();

    //Read the input
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(MODEL_PATH, *model) == -1 || pcl::io::loadPCDFile<pcl::PointXYZ>(SCENE_PATH, *scene) == -1) {
        cout << "Error: Couldn't read file." << endl;
        clog << "Error: Couldn't read file." << '\n';
        return -1;
    }


	//Display initial state
    pcl::visualization::PCLVisualizer init_view("Initial view");
    init_view.addPointCloud<pcl::PointXYZ>(scene, "Scene");
    init_view.addPointCloud<pcl::PointXYZ>(model, "Model");
	init_view.spin();

  //Do the pose estimation using one of the feature extraction functions
  cout << "Extracting Spin Image signatures from the model and scene...  " ;
  cout.flush();

high_resolution_clock::time_point t1 = high_resolution_clock::now();
  Eigen::Matrix4f pose;
  pose = select_feature_extraction(model, scene);
high_resolution_clock::time_point t2 = high_resolution_clock::now();

//auto duration = duration_cast<microseconds>( t2 - t1 ).count();
  //  cout << duration;


	// Apply the pose
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedModel (new pcl::PointCloud<pcl::PointXYZ>);
  //  Eigen::Matrix4f pose;
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


Eigen::Matrix4f select_feature_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
{
  //Eigen::Matrix4f pose;
if (METHOD=="SPINIMAGES")
  return SpinImages(model, scene);
else if (METHOD=="FPFH")
  return FPFH(model, scene);
else if (METHOD=="SHOT")
  return SHOT(model, scene);
//else if (METHOD=="USC")
//  return = USC(model, scene);
else
	cout << "Error: The requested method specified in the settings file does not exist." << endl;

}
