#include "3DModellingH.h"
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

class ICPAlgorithm
{
public:
	ICPAlgorithm();
	~ICPAlgorithm();
	void register_with_previous_aligned(vector<PointCloudT> &clouds, int iteration, double probability);
	void register_with_result(vector<PointCloudT> &clouds, int iteration, double probability);
	void build_3d_map(vector<PointCloudT> &clouds, int iteration, double probability);
	void aligning_two_pointcloud(const PointCloudT &src, const PointCloudT &tgt, PointCloudT &aligned, int iteration, double probability, Matrix4f &total_transformation);
	void transform_pointcloud(const PointCloudT & in, PointCloudT & out, const Matrix4f & transformation);
	void compute_rigid_transformation(const PointCloudT &source, const PointCloudT &target, Matrix4f &transformation);
	void find_nearest_neighbors(const PointCloudT &pc_a, const PointCloudT &pc_b, PointCloudT &p, PointCloudT &q);
	void extract_rotation_and_translation(const Matrix4f &transformation, Matrix3f &rotation, Vector3f &translation);
	void get_transform_matrix(const Matrix3f & rotation, const Vector3f & translation, Matrix4f &transformation);
	void get_random_points(const PointCloudT &cloud_in, PointCloudT &subsample, double probability);
	void subtract_centroid(const PointCloudT &cloud_input, const Vector4f &centroid, MatrixXf &cloud_output);
	void calculate_centroid(const PointCloudT &cloud_input, Vector4f &centroid);

private:

};