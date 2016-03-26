//#include "3DModellingH.h"
//#include "FileProcessingH.h"
//
//class SimpleOpenNIViewer
//{
//public:
//	pcl::visualization::CloudViewer viewer;
//	pcl::Grabber* interface;
//	FileProcessing file;
//	bool pcd_flag;
//	bool ply_flag;
//	bool flag;
//	int counter;
//
//	SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer")
//	{
//		this->interface = new pcl::io::OpenNI2Grabber();
//		this->pcd_flag = true;
//		this->ply_flag = true;
//		this->flag = true;
//		this->counter = 0;
//	}
//
//	PointCloudConstPtr cloud2;
//	PointCloudPtr pc_ptr;
//	PointCloudPtr cloud_working_copy;
//
//	void cloud_cb_(PointCloudConstPtr &cloud)
//	{
//		PointCloudPtr cloud_out(new MyPointCloud);
//		std::vector<int> mapping;
//		PointCloudPtr cloud_in(new MyPointCloud);
//		copyPointCloud(*cloud, *cloud_in);
//		pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, mapping);
//		if (!viewer.wasStopped())
//			viewer.showCloud(cloud_out);
//	}
//
//	void run()
//	{
//		boost::function<void(PointCloudConstPtr&)> f =
//			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
//
//		this->interface->registerCallback(f);
//
//		this->interface->start();
//
//		while (!viewer.wasStopped())
//			boost::this_thread::sleep(boost::posix_time::seconds(1));
//
//		this->interface->stop();
//	}
//
//};
//
//int main()
//{
//	SimpleOpenNIViewer v;
//	v.run();
//	//double *arr = new double[10000000];
//	//bool format = true;
//	//bool use_camera = true;
//	//// Load the first file
//
//	//pcl::PCLPointCloud2 cloud;
//	//if (!loadCloud("test_pcd.pcd", cloud))
//	//	return (-1);
//
//	//// Convert to PLY and save
//	//saveCloud("test_ply.ply", cloud, format, use_camera);
//	//cout << "DONE !" << endl;
//
//	return 0;
//}