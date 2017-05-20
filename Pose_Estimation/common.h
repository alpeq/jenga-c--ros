#ifndef COMMON_H
#define COMMON_H

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/random.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <chrono>

using namespace std::chrono;
using namespace std;
using namespace pcl;
using std::cout;
using std::ifstream;
using std::string;

// for timestamps
time_t _tm =time(NULL );
struct tm * curtime = localtime ( &_tm );

//Ouput file stream for the logging
ofstream ofs("../PoseEstimation_Log.txt",ios_base::app| ios_base::out);

//initializing parameter settings
string SCENE_PATH="\0";
std::vector<std::string> pcds;
string MODEL_PATH="\0";
string METHOD="\0";
float NORM_RADIUS = 0.0;
float FEATURE_RADIUS = 0.0;
float RANSAC_ITR = 0.0;
float INLIER_TRSH = 0.0;
float LEAFSIZE = 0.0;



/***** This file contains the following functions *****/
pcl::PointCloud<pcl::Normal> computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float normRadius = -1);
Eigen::Matrix4f RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::Correspondences corr, int ransacItr = -1, int inlierTrsh = -1);
void DownSampler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
void Load_Settings(); //function definition placeholder
float user_input();
string ReplaceStringInPlace(std::string& subject, const std::string& search,const std::string& replace);

//Compute normals function
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

//Ransac
Eigen::Matrix4f RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::Correspondences corr, int ransacItr, int inlierTrsh) {
	cout << "Starting RANSAC..." << '\n';
  cout.flush();

	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

	while (true)
	{
	//Prepare for RANSAC
    	pcl::search::KdTree<pcl::PointXYZ> scene_tree;			//Create a k-d tree for scene
    	scene_tree.setInputCloud(scene);
	//Create Matrix to save the pose
	PointCloud<pcl::PointXYZ>::Ptr model_aligned(new pcl::PointCloud<pcl::PointXYZ>);		//Create a new point cloud for the aligned result
	// Start RANSAC
	float penalty = FLT_MAX;
	int RANSACS=0;
	size_t INLIERS = 0;

	pcl::common::UniformGenerator<int> gen(0, corr.size() - 1);	//UniformGenerator produces random value

	for(size_t i = 0; i < RANSAC_ITR; ++i) {
		if((i + 1) % 100 == 0)
			cout << "\t" << "Iteration " << i+1 << " of " << RANSAC_ITR << endl;

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
			INLIERS= inliers;
			RANSACS=i;
		}
	}

	float RANSACSCORE = (INLIERS/(INLIER_TRSH*LEAFSIZE)/pow(10,6));
	// Apply best pose found
	cout << "\t" << "Applying best pose with inliers: " << INLIERS << '\n' << "\n";
	cout << "\t" << "Score: "<<RANSACSCORE<< " (inliers / threshold * C_downsampling) \n";
	cout << "\t" << "Overall Score: "<<RANSACSCORE<< " (inliers / threshold * C_downsampling) \n";
	clog << "Iterations of RANSAC = \t" << RANSACS <<" yields:\n" << "\n";
	clog << "\t" << "Pose inliers = "<<INLIERS<<"\n";
	clog << "\t" << "Score        = "<<RANSACSCORE<< "\t\t (inliers / threshold * C_downsampling) \n" << "\n";

	cout.flush();
	clog.flush();

	transformPointCloud(*model, *model_aligned, pose);


	//View result
	visualization::PCLVisualizer result_viewer("Result");
	//visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (model_aligned, 0, 255, 0);
	result_viewer.addPointCloud<PointXYZ>(model_aligned, "Model aligned");
	result_viewer.addPointCloud<PointXYZ>(scene, "Scene");

	result_viewer.spin();

	cout << "<------------------------------------------->\n \n";


	string input=("1");
	//int NEW_RANSAC_ITR=0;
	cout << "Input new Ransac Inlier threshold (float), or Press [0] to exit\n";

	//getline(cin, input);

	float NEW_INLIER_TRSH =1.0;
	NEW_INLIER_TRSH = user_input();
	//NEW_INLIER_TRSH = stof(input.c_str());
	if (NEW_INLIER_TRSH == 0.0)
	{
	  cout << "exiting...\n";
	  cout.flush();
	  sleep(1);
	  break;
	}

	cout <<  "How many Ransac iterations? (int) \n";
			float NEW_RANSAC_ITR= user_input();
	clog << "Reusing Spin Images with Inlier threshold of: \n\t" << NEW_INLIER_TRSH << " and " <<NEW_RANSAC_ITR << " ransac iterations \n";
	cout.flush();
	clog.flush();

	INLIER_TRSH=NEW_INLIER_TRSH;
	RANSAC_ITR=NEW_RANSAC_ITR;

	}

	return pose;

}

void DownSampler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) {
  //Reduce the number of points in the cloud to increase the computational speed of the following algorithms
  pcl::VoxelGrid<pcl::PointXYZ> downSampler;            //Create a downsampler object
  downSampler.setInputCloud (cloud_in);
  downSampler.setLeafSize (LEAFSIZE,LEAFSIZE ,LEAFSIZE );
  downSampler.filter(*cloud_out);
}

void Load_Settings() {
	string path = "../settings.txt";
	ifstream fin;                          // Declaring an input stream object
	fin.open(path);                        // Open the file //path.c_str()
	if(fin.is_open())                      // If it opened successfully
	{
		fin >> NORM_RADIUS >> FEATURE_RADIUS >> RANSAC_ITR
		 		>> INLIER_TRSH >> MODEL_PATH >> METHOD >> LEAFSIZE;  // Read the values and
	// store them in these variables

	string file_paths = ReplaceStringInPlace(MODEL_PATH, ".pcd", ".ini");
	ifstream fin2;

	fin2.open(file_paths);
	if(fin2.is_open())
	{
		while (true){
			getline(fin2, path);
			if (path.compare("end") == 0)
					break;
			else {
				pcds.push_back(path);
						}
					}
			fin2.close();
string BackToOriginal = ReplaceStringInPlace(MODEL_PATH, ".ini", ".pcd");
				}
		else {
			cerr << "Error opening .ini file" << '\n';
		}

		fin.close();

	}

	cout << "Done!\n";
	//current run settings
		cout << "<---------------[SETTINGS]---------------->\n"; //report settings used to console
		cout << asctime(curtime); // "\n"
		cout << "Norm Radius:    " << NORM_RADIUS << '\n';
		cout << "Feature Radius:    " << FEATURE_RADIUS << '\n';
		cout << "Feature Method: "<< METHOD << "\n";
		cout << "Model:      " << MODEL_PATH << "\n";
		cout << "Ransac itr:     "<< RANSAC_ITR << "\n";
		cout << "Inlier +/-:     "<< INLIER_TRSH << "\n";
		cout << "Leafsize:       "<< LEAFSIZE << "\n\n";

		cout.flush();

		// Send similar info to log file
		clog << "<--------------------------------------------------------------------------------------------------------------- " << asctime(curtime);
		clog << "Feature Method: "<< METHOD << "\n";
		clog << "Norm Radius: " << NORM_RADIUS << '\n';
		clog << "Feature Radius: " << FEATURE_RADIUS << '\n';
		clog << "Model:       " << MODEL_PATH << "\n";
		clog << "Ransac itr:     "<< RANSAC_ITR << "\n";
		clog << "Inlier +/-:     "<< INLIER_TRSH << "\n";
		clog << "Leafsize:    "<< LEAFSIZE << "\n\n";
		clog.flush();

}

void Present_and_Report(high_resolution_clock::time_point t1,pcl::PointCloud<pcl::PointXYZ>::Ptr model, pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::Correspondences corr, int spinMatches) {
// Remember to add before extraction and matching high_resolution_clock::time_point t1 = high_resolution_clock::now();
 high_resolution_clock::time_point t2 = high_resolution_clock::now();
 auto duration = duration_cast<microseconds>( t2 - t1 ).count();

 // Show matches
	cout <<"Done!"<< '\n';
visualization::PCLVisualizer match_viewer("Matches");
match_viewer.addPointCloud<PointXYZ>(model, "Model");
match_viewer.addPointCloud<PointXYZ>(scene, "Scene");
match_viewer.addCorrespondences<PointXYZ>(model, scene, corr, 1);
match_viewer.spin();

//report number of matches to console and log
 cout << METHOD <<" took: "<<duration/1000.0 <<" miliseconds\n";
 cout << METHOD << " Found " << spinMatches << " matches!\n";
 clog << METHOD <<" took: "<<duration/1000.0 <<" miliseconds\n";
 clog << METHOD << " Found " << spinMatches << " matches!\n";
 cout <<"Done!" <<'\n';
 cout.flush();
 clog.flush();
}


//This function does not work as intended now, but does not harm either, we leave it for future improvement

float user_input() {
	float user_in;
	string str;
	getline(cin, str);
	try {
		user_in = stof(str.c_str());
	}
	catch (string user_in) {
		return user_input();
	}
	return user_in;
}

string ReplaceStringInPlace(std::string& subject, const std::string& search,const std::string& replace)
{
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
         subject.replace(pos, search.length(), replace);
         pos += replace.length();
         return subject;
    }
}

#endif
