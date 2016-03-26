#include "3DModellingH.h"
#include "FileProcessingH.h"

FileProcessing::FileProcessing()
{
}

FileProcessing::~FileProcessing()
{
}

bool 
FileProcessing::loadCloud(const string & filename, PCLPointCloud2 & cloud)
{
	TicToc tt;

	tt.tic();
	if (loadPCDFile(filename, cloud) < 0)
		return (false);
	print_info("[done, ");
	print_value("%g", tt.toc());
	print_info(" ms : ");
	print_value("%d", cloud.width * cloud.height);
	print_info(" points]\n");
	print_info("Available dimensions: ");
	print_value("%s\n", pcl::getFieldsList(cloud).c_str());

	return (true);
}

void 
FileProcessing::saveCloud(const string & filename, const PCLPointCloud2 & cloud, bool binary, bool use_camera)
{
	TicToc tt;
	tt.tic();

	print_highlight("Saving ");
	print_value("%s ", filename.c_str());

	pcl::PLYWriter writer;
	writer.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary, use_camera);

	print_info("[done, ");
	print_value("%g", tt.toc()); print_info(" ms : ");
	print_value("%d", cloud.width * cloud.height);
	print_info(" points]\n");
}
