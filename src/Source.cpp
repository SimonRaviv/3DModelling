//#include <iostream>
//#include <string>
//#include <vector>
//#include <stdio.h>
//#include <sstream>
//#include <stdlib.h>
//
//#include <pcl/console/time.h>   // TicToc 
//#include <pcl/console/parse.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//using std::cout;
//using std::endl;
//using namespace std;
//
//
//int
//main(int argc,
//	char* argv[])
//{
//	vector<PointCloudT::Ptr> clouds;
//	vector<PointCloudT::Ptr>::iterator it;
//	it = clouds.begin();
//	PointCloudT::Ptr cloud_source_registered(new PointCloudT);
//	PointCloudT::Ptr cloud(new PointCloudT);
//	PointCloudT::Ptr result(new PointCloudT);
//	pcl::IterativeClosestPoint<PointT, PointT> icp;
//	string file_name;
//
//	for (size_t i = 0; i < 19; i++)
//	{
//		file_name = "C:\\3DModelling\\Build\\PointCloud" + std::to_string(i) + ".ply";
//		pcl::io::loadPLYFile<PointT>(file_name, *cloud);
//		it = clouds.insert(it, cloud);
//	}
//
//	cout << "Data loaded" << endl;
//
//	*result += *clouds.at(0);
//
//	for (size_t i = 0; i < 15; i++)
//	{
//		// Set the input source and target
//		icp.setInputCloud(clouds.at(i));
//		icp.setInputTarget(clouds.at(i + 1));
//		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
//		icp.setMaxCorrespondenceDistance(0.05);
//		// Set the maximum number of iterations (criterion 1)
//		icp.setMaximumIterations(50);
//		// Set the transformation epsilon (criterion 2)
//		icp.setTransformationEpsilon(1e-8);
//		// Set the euclidean distance difference epsilon (criterion 3)
//		icp.setEuclideanFitnessEpsilon(1);
//		// Perform the alignment
//		icp.align(*cloud_source_registered);
//
//		cout << "ICP " << i << "done !" << endl;
//		*result += *cloud_source_registered;
//
//	}
//	pcl::io::savePLYFile("C:\\3DModelling\\Build\\Final.ply", *result);
//
//	cout << "File saved !" << endl;
//
//	// Obtain the transformation that aligned cloud_source to cloud_source_registered
//	Eigen::Matrix4f transformation = icp.getFinalTransformation();
//
//	cout << transformation << endl;
//
//	return(0);
//}