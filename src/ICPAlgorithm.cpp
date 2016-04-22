#include "ICPAlgorithm.h"
#include "FileProcessingH.h"
ICPAlgorithm::ICPAlgorithm()
{
}

ICPAlgorithm::~ICPAlgorithm()
{
}
//TODO:
void ICPAlgorithm::Register(vector<PointCloudT> &clouds)
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
	for (size_t i = 0; i < clouds.size(); i++)
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

void ICPAlgorithm::build_3d_map(vector<PointCloudT>& clouds, int iteration, double probability)
{
	PointCloudPtr result_show(new PointCloudT);
	PointCloudT result;
	PointCloudPtr target(new PointCloudT);
	PointCloudPtr source(new PointCloudT);
	PointCloudPtr source_trans_PointCloud(new PointCloudT);
	Matrix4f last_transformation;
	FileProcessing fp;
	visualization::CloudViewer viewer("PCL OpenNI Viewer");
	last_transformation.setIdentity();
	result += clouds[0];
	//result_show.reset();
	result_show->header = result.header;
	result_show->points.resize(result.points.size());
	*source_trans_PointCloud = result;
	*target = clouds[0];
	for (size_t i = 0; i < clouds.size(); i++)
	{

		*source = clouds[i + 1];
		transform_pointcloud(*source, *source, last_transformation);
		aligning_two_pointcloud(*source, *target, *source_trans_PointCloud, iteration, probability, last_transformation);
		*target = *source_trans_PointCloud;

		result += *source_trans_PointCloud;
		*result_show = result;
		viewer.showCloud(result_show);
		PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
		PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
		toPCLPointCloud2(result, *cloud);
		VoxelGrid<PCLPointCloud2> sor;
		sor.setInputCloud(cloud);
		sor.setLeafSize(0.005f, 0.005f, 0.005f);
		sor.filter(*cloud_filtered);
		pcl::fromPCLPointCloud2(*cloud_filtered, result);

	}
	//get_random_points(result, result, 0.01);
	fp.save_point_cloud("FullSciene.ply", result);

}

//checked!
void ICPAlgorithm::aligning_two_pointcloud(const PointCloudT & src, const PointCloudT & tgt, PointCloudT & aligned, int iteration, double probability, Matrix4f &total_transformation)
{
	PointCloudPtr sub_source(new PointCloudT);
	PointCloudPtr temp_source(new PointCloudT);
	PointCloudPtr temp_target(new PointCloudT);
	//get_random_points(tgt, aligned, probability);
	get_random_points(tgt, aligned, 1);

	// for the total transformation matrix
	total_transformation = total_transformation.setIdentity();

	transform_pointcloud(tgt, *temp_target, total_transformation);
	transform_pointcloud(src, *temp_source, total_transformation);
	Matrix3f rot;
	Vector3f trans;
	pcl::console::TicToc time;
	time.tic();
	get_random_points(*temp_source, *sub_source, probability);
	extract_rotation_and_translation(total_transformation, rot, trans);
	// Apply the previous transformations to b so that it is positioned near
	// the last accumulated points
	transform_pointcloud(*sub_source, *sub_source, total_transformation);

	PointCloudPtr p(new PointCloudT);
	PointCloudPtr q(new PointCloudT);

	// ICP iteration START here
	for (int i = 0; i < iteration; ++i) {
		cout << "ICP Iteration " << i << endl;
		// Find nearest neighbor usin kdtree of two point clouds.
		find_nearest_neighbors(aligned, *sub_source, *p, *q);

		// getting the  transformation matrix from current frame to the previous frame.
		Matrix4f transformQtoP;
		compute_rigid_transformation(*q, *p, transformQtoP);
		cout << transformQtoP << endl;

		// transform source to target given the tranformation.
		// Q'=Q*R+T
		transform_pointcloud(*sub_source, *sub_source, transformQtoP);
		transform_pointcloud(*temp_source, *temp_source, transformQtoP);
		// updating the total transformation.
		total_transformation *= transformQtoP;

	}
	cout << "complete " << time.toc() << " ms" << endl;
	// merging two pointclouds.
	PointCloudPtr combined(new PointCloudT);
	*combined += *temp_target;
	*combined += *temp_source;

	//preparing the return value.

	aligned = *temp_source;
	//save the result as ply file.

	//FileProcessing fp;
	//fp.save_point_cloud("output.ply", *combined);


}
//checked!
void ICPAlgorithm::transform_pointcloud(const PointCloudT & in, PointCloudT & out, const Matrix4f & transformation)
{
	Matrix3f rotation;
	Vector3f translation;

	// initial the output pointcloud if both pointcloud are not the same
	if (&in != &out)
	{
		out.header = in.header;
		out.width = in.width;
		out.height = in.height;
		out.is_dense = in.is_dense;
		out.points.reserve(out.points.size());
		out.points.assign(in.points.begin(), in.points.end());
	}

	// extracting the translation and rotation matrix out of 4x4 Transformation Matrix.
	extract_rotation_and_translation(transformation, rotation, translation);
	// applying the transformation on the pointcloud.
	for (size_t i = 0; i < out.points.size(); ++i)
		out.points[i].getVector3fMap() = rotation * in.points[i].getVector3fMap() + translation;
}
//checked!
void ICPAlgorithm::compute_rigid_transformation(const PointCloudT &source, const PointCloudT &target, Matrix4f &transformation)
{
	Vector4f centroid_source, centroid_target;
	MatrixXf subtracted_centroid_source, subtracted_centroid_target;
	Matrix3f covarianse, U, V, rotation;
	Vector3f rotated_centroid;
	// initial the transformation as identity.
	transformation.setIdentity();

	// find the centroids of the given pointcloud.
	calculate_centroid(source, centroid_source);
	calculate_centroid(target, centroid_target);

	// eleminate the centroid from the pointclouds
	subtract_centroid(source, centroid_source, subtracted_centroid_source);

	subtract_centroid(target, centroid_target, subtracted_centroid_target);

	// compute Covarians matrix subtracted_mean_source * subtracted_mean_target'
	covarianse = (subtracted_centroid_source * subtracted_centroid_target.transpose()).topLeftCorner<3, 3>();

	// apply SVD on covarians matrix.
	JacobiSVD<Matrix3f> SVD(covarianse, ComputeFullU | ComputeFullV);
	U = SVD.matrixU();
	V = SVD.matrixV();

	//if reflection detected.
	if (U.determinant() * V.determinant() < 0)
		for (int z = 0; z < 3; ++z)
			V(z, 2) *= -1;

	//The rotation matrix :𝑅 = 𝑉∗𝑈^𝑇
	rotation = V * U.transpose();

	// Return the correct transformation
	transformation.topLeftCorner<3, 3>() = rotation;
	rotated_centroid = rotation * centroid_source.head<3>();
	//The translation matrix : 𝑡 =  𝑐𝑒𝑛𝑡𝑟𝑜𝑖𝑑_(𝑃^∗)− 𝑅∗𝑐𝑒𝑛𝑡𝑟𝑜𝑖𝑑_𝑄.
	transformation.block <3, 1>(0, 3) = centroid_target.head<3>() - rotated_centroid;
}
//checked!
void ICPAlgorithm::find_nearest_neighbors(const PointCloudT & prev_frame, const PointCloudT & curr_frame, PointCloudT & p, PointCloudT & q)
{
	int K = 1;
	KdTreeFLANN<PointT> kdtree;
	PointCloudPtr prev(new PointCloudT);
	PointCloudPtr curr(new PointCloudT);
	PointT search_point, matched_point;
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
void ICPAlgorithm::extract_rotation_and_translation(const Matrix4f &transformation, Matrix3f &rotation, Vector3f &translation)
{
	rotation = transformation.block<3, 3>(0, 0);
	translation = transformation.block<3, 1>(0, 3);
}
//checked!
void ICPAlgorithm::get_transform_matrix(const Matrix3f & rotation, const Vector3f & translation, Matrix4f &transformation)
{
	transformation.setIdentity();
	transformation.block<3, 3>(0, 0) = rotation;
	transformation.block<3, 1>(0, 3) = translation;
}
//checked!
void ICPAlgorithm::get_random_points(const PointCloudT & cloud_in, PointCloudT & subsample, double probability)
{
	int j = 0;
	std::vector<int> index;

	subsample.is_dense = true; // there are non NANs in the subsample cloud.

	// initial the subsample pointcloud if both pointcloud are not the same
	if (&cloud_in != &subsample)
	{
		subsample.header = cloud_in.header;
		subsample.points.resize(cloud_in.points.size());
	}
	// initialize the index size
	index.resize(cloud_in.points.size());
	srand(time(NULL));
	for (size_t i = 0; i < subsample.points.size(); ++i)
	{

		if (((rand() % 100) / 100.0) < probability)
		{
			subsample.points[j] = cloud_in.points[i];
			index[j] = i;
			j++;
		}

	}

	// update subsample to rigth size
	subsample.points.resize(j);
	index.resize(j);
	subsample.height = 1;
	subsample.width = j;

	cout << "before subsample: " << cloud_in.points.size() << endl;
	cout << "subsample: " << subsample.points.size() << endl;
}
//checked!
void ICPAlgorithm::subtract_centroid(const PointCloudT &cloud_input, const Vector4f &centroid, MatrixXf &cloud_output)
{
	size_t points_number;

	points_number = cloud_input.points.size();
	cloud_output = MatrixXf::Zero(4, points_number);        // initial the output matrix

	for (size_t i = 0; i < points_number; ++i)
		cloud_output.block<4, 1>(0, i) = cloud_input.points[i].getVector4fMap() - centroid;

	// reset the forth dimention
	cloud_output.block(3, 0, 1, points_number).setZero();
}
//checked!
void ICPAlgorithm::calculate_centroid(const PointCloudT &cloud_input, Vector4f &centroid)
{
	centroid.setZero(); //initial centroid
	if (cloud_input.points.empty()) // if cloud is empty
		return;

	for (size_t i = 0; i < cloud_input.points.size(); ++i)
		centroid += cloud_input.points[i].getVector4fMap();
	centroid[3] = 0; //reset the forth dimention
	centroid = centroid / cloud_input.points.size();
}
