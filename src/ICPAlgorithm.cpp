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

void ICPAlgorithm::Register(vector<PointCloudT> &clouds, int FrameNumber)
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

		viewer.showCloud(result);

	}
	pcl::io::savePLYFile("C:\\3DModelling\\Build\\Final.ply", *result);


	cout << "File saved !" << endl;

	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	Eigen::Matrix4f transformation = icp.getFinalTransformation();

	cout << transformation << endl;
}

void ICPAlgorithm::transform_pointcloud(const PointCloudT & cloud_in, PointCloudT & cloud_out, const Matrix4f & transform)
{
	// If the clouds are not the same, prepare the output
	if (&cloud_in != &cloud_out)
	{
		cloud_out.header = cloud_in.header;
		cloud_out.width = cloud_in.width;
		cloud_out.height = cloud_in.height;
		cloud_out.is_dense = cloud_in.is_dense;
		cloud_out.points.reserve(cloud_out.points.size());
		cloud_out.points.assign(cloud_in.points.begin(), cloud_in.points.end());
	}

	Matrix3f rot = transform.block<3, 3>(0, 0);
	Vector3f trans = transform.block<3, 1>(0, 3);

	for (size_t i = 0; i < cloud_out.points.size(); ++i)
		cloud_out.points[i].getVector3fMap() = rot * cloud_in.points[i].getVector3fMap() + trans;
}

void ICPAlgorithm::rigid_transform_3D(const PointCloudT & cloud_src, const PointCloudT & cloud_tgt, Matrix4f & transformation)
{
	// <cloud_src,cloud_src> is the source dataset
	transformation.setIdentity();

	Vector4f centroid_src, centroid_tgt;
	// Estimate the centroids of source, target
	compute_centroid(cloud_src, centroid_src);
	compute_centroid(cloud_tgt, centroid_tgt);

	// Subtract the centroids from source, target
	MatrixXf cloud_src_demean;
	subtract_centroid(cloud_src, centroid_src, cloud_src_demean);

	MatrixXf cloud_tgt_demean;
	subtract_centroid(cloud_tgt, centroid_tgt, cloud_tgt_demean);

	// Assemble the correlation matrix H = source * target'
	Matrix3f H = (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner<3, 3>();

	// Compute the Singular Value Decomposition
	JacobiSVD<Matrix3f> svd(H, ComputeFullU | ComputeFullV);
	Matrix3f u = svd.matrixU();
	Matrix3f v = svd.matrixV();

	//in case of reflection 
	if (u.determinant() * v.determinant() < 0)
	{
		for (int x = 0; x < 3; ++x)
			v(x, 2) *= -1;
	}
	//The rotation matrix :𝑅 = 𝑉∗𝑈^𝑇
	Matrix3f R = v * u.transpose();

	// Return the correct transformation
	transformation.topLeftCorner<3, 3>() = R;
	Vector3f Rc = R * centroid_src.head<3>();
	//The translation matrix : 𝑡 =  𝑐𝑒𝑛𝑡𝑟𝑜𝑖𝑑_(𝑃^∗)− 𝑅∗𝑐𝑒𝑛𝑡𝑟𝑜𝑖𝑑_𝑄.
	transformation.block <3, 1>(0, 3) = centroid_tgt.head<3>() - Rc;
}

void ICPAlgorithm::save_point_cloud(const string & filename, const PointCloudT &cloud)
{
	TicToc tt;
	tt.tic();
	savePLYFile(filename, cloud);
	print_info("[done, ");
	print_value("%g", tt.toc()); print_info(" ms : ");
	print_value("%d", cloud.width * cloud.height);
	print_info(" points]\n");
}

void ICPAlgorithm::extract_rotation_and_translation(const Matrix4f & transformation, Matrix3f & r, Vector3f & t)
{
	r= transformation.block<3, 3>(0, 0) ;
	t= transformation.block<3, 1>(0, 3) ;
}

void ICPAlgorithm::get_transform_matrix(const Matrix3f & r, const Vector3f & t, Matrix4f &transformation)
{
	transformation.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	transformation.block<3, 3>(0, 0) = r;
	transformation.block<3, 1>(0, 3) = t;
	//transformation.rightCols<1>() = t;
}

void ICPAlgorithm::get_random_points(const PointCloudT & cloud_in, PointCloudT & subsample, double probability)
{
	std::vector<int> index;
	subsample.is_dense = true;

	// If the clouds are not the same, prepare the output
	if (&cloud_in != &subsample)
	{
		subsample.header = cloud_in.header;
		subsample.points.resize(cloud_in.points.size());
	}
	// Reserve enough space for the indices
	index.resize(cloud_in.points.size());
	int j = 0;
	srand(time(NULL));
		for (size_t i = 0; i < subsample.points.size(); ++i)
		{

			if(((rand() % 100) / 100.0) < probability)
			{ 
				subsample.points[j] = cloud_in.points[i];
				index[j] = i;
				j++;
			}

		}

	// Resize to the correct size
	subsample.points.resize(j);
	index.resize(j);
	subsample.height = 1;
	subsample.width = j;
}

void ICPAlgorithm::subtract_centroid(const PointCloudT &cloud_in, const Vector4f &centroid, MatrixXf &cloud_out)
{
	size_t points_number = cloud_in.points.size();

	cloud_out = MatrixXf::Zero(4, points_number);        // keep the data aligned

	for (size_t i = 0; i < points_number; ++i)
		// One column at a time
		cloud_out.block<4, 1>(0, i) = cloud_in.points[i].getVector4fMap() - centroid;

	// Make sure we zero the 4th dimension out (1 row, N columns)
	cloud_out.block(3, 0, 1, points_number).setZero();
}

void ICPAlgorithm::compute_centroid(const PointCloudT &cloud, Vector4f &centroid)
{
	// Initialize to 0
	centroid.setZero();
	if (cloud.points.empty())
		return;

		for (size_t i = 0; i < cloud.points.size(); ++i)
			centroid += cloud.points[i].getVector4fMap();
		centroid[3] = 0;
		centroid /= cloud.points.size();
}
