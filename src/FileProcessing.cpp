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
	PCLPointCloud2 pcl2, PC1, PC2, r;
	int i;
	PointCloudPtr cloud_in(new MyPointCloud);
	PointCloudPtr cloud_out(new MyPointCloud);
	IterativeClosestPoint<PointT, PointT> icp;
	MyPointCloud Final;
	//toPCLPointCloud2((*set)[15], PC2);
	//const pcl::PCLPointCloud2 &cloud1
	// compress point cloud
	cout << "1" << endl;
	*cloud_in = (*set)[2];
	//stringstream compressedData;
	//OctreePointCloudCompression<PointT>* PointCloudEncoder;
	//OctreePointCloudCompression<PointT>* PointCloudDecoder;
	//PointCloudEncoder->encodePointCloud(cloud_in, compressedData);
	//// decompress point cloud
	//PointCloudDecoder->decodePointCloud(compressedData, cloud_in);
	//cout << "1" << endl;
	*cloud_out = (*set)[3];

	//PointCloudEncoder->encodePointCloud(cloud_out, compressedData);
	//// decompress point cloud
	//PointCloudDecoder->decodePointCloud(compressedData, cloud_out);
	cout << "1" << endl;
	icp.setInputCloud(cloud_in);
	cout << "2" << endl;
	icp.setInputTarget(cloud_out);
	cout << "3" << endl;
	icp.align(Final);
	cout <<"4" << endl;
	cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << endl;
	cout << icp.getFinalTransformation() << endl;
	toPCLPointCloud2(Final, PC1);
	//toPCLPointCloud2((*set)[2], PC1);
	concatenatePointCloud(r, PC1, r);

	//for (j = 0, i = 0; j != set->size(); j++, i++)
	//{

	//	*cloud_in = (*set)[j];
	//	*cloud_out = (*set)[j + 1];
	//	icp.setInputCloud(cloud_in);
	//	icp.setInputTarget(cloud_out);
	//	icp.align(Final);

	//	toPCLPointCloud2(Final, PC1);
	//	//toPCLPointCloud2((*set)[2], PC1);
	//	concatenatePointCloud(r, PC1, r);
	//	//file = filename;
	//	//ile += to_string(i) + ".ply";
	//	//toPCLPointCloud2((*set)[j], pcl2);
	//	//saveCloud(file, pcl2, binary, use_camera);

	//}
	saveCloud("r.ply", r, binary, use_camera);

}
