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
ICPAlgorithm::Register(vector<PointCloudT> &clouds,int FrameNumber)
{

	PointCloudPtr cloud_source_registered(new PointCloudT);
	PointCloudPtr result(new PointCloudT);
	PointCloudPtr cloud_in(new PointCloudT);
	PointCloudPtr cloud_out(new PointCloudT);
	visualization::CloudViewer viewer("PCL OpenNI Viewer");
	IterativeClosestPoint<PointT, PointT> icp;


	cout << "Data loaded" << endl;

	*result += clouds[0];

	for (size_t i = 0; i < FrameNumber; i++)
	{
		// Set the input source and target
		*cloud_in = clouds[i];
		*cloud_out = clouds[i+1];
		icp.setInputCloud(cloud_in);
		icp.setInputTarget(cloud_out);
		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance(0.05);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations(50);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon(1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon(1);
		// Perform the alignment
		icp.align(*cloud_source_registered);

		cout << "ICP " << i << "done !" << endl;
		*result += *cloud_source_registered;
		viewer.showCloud(result);

	}
	pcl::io::savePLYFile("C:\\3DModelling\\Build\\Final.ply", *result);

	cout << "File saved !" << endl;

	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	cout << transformation << endl;
}