#include "FileProcessingH.h"

FileProcessing::FileProcessing()
{
}

FileProcessing::~FileProcessing()
{
}
/* Saving point cloud as ply format file. */
void
FileProcessing::save_point_cloud(const string & filename, const PointCloudT &cloud)
{
	savePLYFile(filename, cloud);
	print_info("[done, ");
	print_value("%d", cloud.width * cloud.height);
	print_info(" points]\n");
}