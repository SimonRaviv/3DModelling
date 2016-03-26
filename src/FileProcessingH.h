#include "3DModellingH.h"

class FileProcessing
{
public:
	FileProcessing();
	~FileProcessing();
	bool loadCloud(const string &filename, PCLPointCloud2 &cloud);
	void saveCloud(const string &filename, const PCLPointCloud2 &cloud, bool binary, bool use_camera);

private:

}; 