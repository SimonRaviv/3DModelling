#include "FileProcessingH.h"

FileProcessing::FileProcessing()
{
}

FileProcessing::~FileProcessing()
{
}

void
FileProcessing::save_point_cloud(const string & filename, const PointCloudT &cloud)
{
	//TicToc tt;
	//tt.tic();
	savePLYFile(filename, cloud);
	print_info("[done, ");
	//print_value("%g", tt.toc()); print_info(" ms : ");
	print_value("%d", cloud.width * cloud.height);
	print_info(" points]\n");
}