//#include "3DModellingH.h"
//#include "FileProcessingH.h"
//
//class PointCloudSet
//{
//public:
//	vector<MyPointCloud> set;
//	vector<MyPointCloud>::iterator it;
//
//	PointCloudSet();
//
//	~PointCloudSet();
//
//private:
//
//};
//
//PointCloudSet::PointCloudSet()
//{
//	this->it = set.begin();
//}
//PointCloudSet::~PointCloudSet()
//{
//}
//
//class SimpleOpenNIViewer
//{
//public:
//	pcl::visualization::CloudViewer viewer;
//	pcl::Grabber* interface;
//	FileProcessing file;
//	PointCloudSet point_cloud_list;
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
//	~SimpleOpenNIViewer()
//	{
//
//	}
//
//	void cloud_cb_(PointCloudConstPtr &cloud)
//	{
//		PointCloudPtr cloud_out(new MyPointCloud);
//		std::vector<int> mapping;
//		PointCloudPtr cloud_in(new MyPointCloud);
//		copyPointCloud(*cloud, *cloud_in);
//		pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, mapping);
//		this->point_cloud_list.it = this->point_cloud_list.set.insert(this->point_cloud_list.it, *cloud_out);
//
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
//	
//	vector<MyPointCloud>::size_type j;
//	PCLPointCloud2 pcl2;
//	bool format = true;
//	bool use_camera = true;
//
//	v.file.makePLYFromPointCloudSet("PointCloud", pcl2, format, use_camera, &v.point_cloud_list.set);
//
//	return 0;
//}