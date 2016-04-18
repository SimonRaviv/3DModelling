﻿#include "ICPAlgorithm.h"

ICPAlgorithm::ICPAlgorithm()
{
}

ICPAlgorithm::~ICPAlgorithm()
{
}
//TODO:
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

//checked!
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
	//rot = rot.transpose();
	Vector3f trans = transform.block<3, 1>(0, 3);

	for (size_t i = 0; i < cloud_out.points.size(); ++i)
		cloud_out.points[i].getVector3fMap() = rot * cloud_in.points[i].getVector3fMap() + trans;
}
//checked!
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
//checked!
void ICPAlgorithm::find_nearest_neighbors(const PointCloudT & prev_frame, const PointCloudT & curr_frame, PointCloudT & p, PointCloudT & q)
{
	KdTreeFLANN<PointT> kdtree;
	PointCloudPtr prev(new PointCloudT);
	PointCloudPtr curr(new PointCloudT);
	PointT search_point,matched_point;
	int K = 1;
	vector<int> pointIdxNKNSearch(K);
	vector<float> pointNKNSquaredDistance(K);
	*prev = prev_frame;
	*curr = curr_frame;

	
	if (prev_frame.points.size() < curr_frame.points.size())
	{
		kdtree.setInputCloud(curr);

	if (&p != &prev_frame && &q != &curr_frame)
	{

		p.header = prev_frame.header;
		p.width = prev_frame.width;
		p.height = prev_frame.height;
		p.is_dense = prev_frame.is_dense;
		p.points.reserve(p.points.size());
		p.points.assign(prev_frame.points.begin(), prev_frame.points.end());
		q.header = prev_frame.header;
		q.width = prev_frame.width;
		q.height = prev_frame.height;
		q.is_dense = prev_frame.is_dense;
		q.points.reserve(q.points.size());
		q.points.assign(prev_frame.points.begin(), prev_frame.points.end());
	}


	for (size_t i = 0; i < prev_frame.points.size(); ++i)
	{
		search_point = prev->points[i];
		p.points[i].x = search_point.x;
		p.points[i].y = search_point.y;
		p.points[i].z = search_point.z;

		kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		matched_point = curr->points[pointIdxNKNSearch[0]];
		q.points[i].x = matched_point.x;
		q.points[i].y = matched_point.y;
		q.points[i].z = matched_point.z;

	}
	}
	else 
	{
		kdtree.setInputCloud(prev);
		if (&p != &prev_frame && &q != &curr_frame)
		{

			p.header = curr_frame.header;
			p.width = curr_frame.width;
			p.height = curr_frame.height;
			p.is_dense = curr_frame.is_dense;
			p.points.reserve(p.points.size());
			p.points.assign(curr_frame.points.begin(), curr_frame.points.end());
			q.header = curr_frame.header;
			q.width = curr_frame.width;
			q.height = curr_frame.height;
			q.is_dense = curr_frame.is_dense;
			q.points.reserve(q.points.size());
			q.points.assign(curr_frame.points.begin(), curr_frame.points.end());
		}


		for (size_t i = 0; i < curr_frame.points.size(); ++i)
		{
			search_point = curr->points[i];
			q.points[i].x = search_point.x;
			q.points[i].y = search_point.y;
			q.points[i].z = search_point.z;

			kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance);
			matched_point = prev->points[pointIdxNKNSearch[0]];
			p.points[i].x = matched_point.x;
			p.points[i].y = matched_point.y;
			p.points[i].z = matched_point.z;

		}
	}

}
//checked!
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
//checked!
void ICPAlgorithm::extract_rotation_and_translation(const Matrix4f & transformation, Matrix3f & r, Vector3f & t)
{
	r= transformation.block<3, 3>(0, 0) ;
	t= transformation.block<3, 1>(0, 3) ;
}
//checked!
void ICPAlgorithm::get_transform_matrix(const Matrix3f & r, const Vector3f & t, Matrix4f &transformation)
{
	transformation.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
	transformation.block<3, 3>(0, 0) = r;
	transformation.block<3, 1>(0, 3) = t;
	//transformation.rightCols<1>() = t;
}
//checked!
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
//checked!
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
//checked!
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
