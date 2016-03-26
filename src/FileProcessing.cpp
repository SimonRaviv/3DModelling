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


void
FileProcessing::makePLYFromPointCloudSet(const string &filename, const PCLPointCloud2 &cloud, bool binary, bool use_camera, vector<MyPointCloud> *set)
{
	string file;
	vector<MyPointCloud>::size_type j;
	PCLPointCloud2 pcl2;
	int i;

	for (j = 0, i = 0; j != set->size(); j++, i++)
	{
		file = filename;
		file += to_string(i) + ".ply";
		toPCLPointCloud2((*set)[j], pcl2);
		saveCloud(file, pcl2, binary, use_camera);
	}
}
