#include "3DModellingH.h"
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

class ICPAlgorithm
{
public:
	ICPAlgorithm();
	~ICPAlgorithm();
	void Register(vector<PointCloudT> &set, int FrameNumber);
	void transform_pointcloud(const PointCloudT &cloud_in, PointCloudT &cloud_out, const Matrix4f &transform);
	void rigid_transform_3D(const PointCloudT &cloud_src, const PointCloudT &cloud_tgt, Matrix4f &transformation);
	//void find_nearest_neighbors(const PointCloudT &pc_a, const PointCloudT &pc_b, PointCloudT &p, PointCloudT &q);
	void save_point_cloud(const string & filename, const PointCloudT &cloud);
	void extract_rotation_and_translation(const Matrix4f &m, Matrix3f &r, Vector3f &transformation);
	void get_transform_matrix(const Matrix3f & r, const Vector3f & t, Matrix4f &transformation);
	void get_random_points(const PointCloudT &cloud_in, PointCloudT &subsample, double probability);
	void subtract_centroid(const PointCloudT &cloud_in, const Vector4f &centroid, MatrixXf &cloud_out);
	void compute_centroid(const PointCloudT &cloud, Vector4f &centroid);

private:

};