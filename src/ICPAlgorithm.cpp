#include "ICPAlgorithm.h"


using std::cout;
using std::endl;
using namespace std;


ICPAlgorithm::ICPAlgorithm()
{
}

ICPAlgorithm::~ICPAlgorithm()
{
}

void
ICPAlgorithm::Register(vector<PointCloudT> &clouds, int FrameNumber)
{

	PointCloudPtr cloud_source_registered(new PointCloudT);
	PointCloudPtr previous_trans_PointCloud(new PointCloudT);
	PointCloudPtr result(new PointCloudT);
	PointCloudPtr cloud_in(new PointCloudT);
	PointCloudPtr cloud_out(new PointCloudT);
	visualization::CloudViewer viewer("PCL OpenNI Viewer");
	IterativeClosestPoint<PointT, PointT> icp;

	cout << "Data loaded" << endl;

	*result += clouds[0];
	*previous_trans_PointCloud = *result;
	for (size_t i = 0; i < FrameNumber; i++)
	{
		// Set the input source and target
		*cloud_in = clouds[i + 1];
		//*cloud_out = clouds[2];
		icp.setInputCloud(cloud_in);
		icp.setInputTarget(previous_trans_PointCloud);
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance(0.005);
		// Set the maximum number of iterations (criterion 1)
		//icp.setMaximumIterations(50);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(1e-11);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(0.5);
		// Perform the alignment
		icp.align(*cloud_source_registered);
		*previous_trans_PointCloud = *cloud_source_registered;
		cout << "ICP " "done !" << endl;
		*result += *cloud_source_registered;
		pcl::VoxelGrid()
		viewer.showCloud(result);

	}
	pcl::io::savePLYFile("C:\\3DModelling\\Build\\Final.ply", *result);


	cout << "File saved !" << endl;

	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	cout << transformation << endl;
}