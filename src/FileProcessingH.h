#include "3DModellingH.h"

class FileProcessing
{
public:
	FileProcessing();
	~FileProcessing();
	bool loadCloud(const string &filename, PCLPointCloud2 &cloud);
	void save_point_cloud(const string & filename, const PointCloudT &cloud);
	void makePLYFromPointCloudSet(const string &filename, const PCLPointCloud2 &cloud, bool binary, bool use_camera, vector<PointCloudT> *set);

private:

}; 