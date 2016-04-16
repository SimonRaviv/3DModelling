#include "3DModellingH.h"
#include <string>
#include <vector>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
class ICPAlgorithm
{
public:
	ICPAlgorithm();
	~ICPAlgorithm();
	void Register(vector<PointCloudT> &set,int FrameNumber);

private:

};