#include "3DModellingH.h"
#include "FileProcessingH.h"

class SimpleOpenNIViewer
{
public:
	pcl::visualization::CloudViewer viewer;
	pcl::Grabber* interface;
	FileProcessing file;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudEncoder;
	pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>* PointCloudDecoder;
	bool pcd_flag;
	bool ply_flag;
	bool flag;
	int counter;

	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer")
	{
		this->interface = new pcl::io::OpenNI2Grabber();
		this->pcd_flag = true;
		this->ply_flag = true;
		this->flag = true;
		this->counter = 0;
	}

	PointCloudConstPtr cloud2;
	PointCloudPtr pc_ptr;
	PointCloudPtr cloud_working_copy;

	void cloud_cb_(PointCloudConstPtr &cloud)
	{
		if (!viewer.wasStopped())
			viewer.showCloud(cloud);
		// stringstream to store compressed point cloud
		std::stringstream compressedData;
		// output pointcloud
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGBA>());

		// compress point cloud
		PointCloudEncoder->encodePointCloud(cloud, compressedData);

		// decompress point cloud
		PointCloudDecoder->decodePointCloud(compressedData, cloudOut);


		// show decompressed point cloud
		viewer.showCloud(cloudOut);
	}

	void run()
	{
		bool showStatistics = false;

		// for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
		pcl::io::compression_Profiles_e compressionProfile = pcl::io::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

		// instantiate point cloud compression for encoding and decoding
		PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>(compressionProfile, showStatistics);
		PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGBA>();

		boost::function<void(PointCloudConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		this->interface->registerCallback(f);

		this->interface->start();

		while (!viewer.wasStopped())
			boost::this_thread::sleep(boost::posix_time::seconds(1));

		this->interface->stop();

		// delete point cloud compression instances
		delete (PointCloudEncoder);
		delete (PointCloudDecoder);

	}

};

int main()
{
	SimpleOpenNIViewer v;
	v.run();
	//double *arr = new double[10000000];
	//bool format = true;
	//bool use_camera = true;
	//// Load the first file

	//pcl::PCLPointCloud2 cloud;
	//if (!loadCloud("test_pcd.pcd", cloud))
	//	return (-1);

	//// Convert to PLY and save
	//saveCloud("test_ply.ply", cloud, format, use_camera);
	//cout << "DONE !" << endl;

	return 0;
}