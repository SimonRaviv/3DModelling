//
//#include "3DModellingH.h"
//#include "FileProcessingH.h"
//#include "ICPAlgorithm.h"
//
//class PointCloudSet
//{
//public:
//	vector<PointCloudT> set;
//	vector<PointCloudT>::iterator it;
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
//	ICPAlgorithm icp_Alg;
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
//	void cloud_cb_(PointCloudConstPtr &cloud)
//	{
//
//		//PointCloudPtr cloud_out(new PointCloudT);
//		//std::vector<int> mapping;
//		//PointCloudPtr cloud_in(new PointCloudT);
//		//copyPointCloud(*cloud, *cloud_in);
//		this->point_cloud_list.it = this->point_cloud_list.set.insert(this->point_cloud_list.it, *cloud);
//		//pcl::io::savePLYFile("file" + std::to_string(frameNumber++) + ".ply", *cloud_out);
//		if (!viewer.wasStopped())
//			viewer.showCloud(cloud);
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
//	cout << "*** Video capturing started ***" << endl;
//	SimpleOpenNIViewer v;
//	v.run();
//	FileProcessing fp;
//	vector<PointCloudT>::size_type j;
//	vector<int> mapping;
//	cout << "*** Video capturing stopped ***" << endl;
//	//PCLPointCloud2 pcl2;
//	//bool format = true;
//	//bool use_camera = true;
//
//	//v.file.makePLYFromPointCloudSet("PointCloud", pcl2, format, use_camera, &v.point_cloud_list.set);
//
//	//for (j = 0; j != v.point_cloud_list.set.size(); j++)
//	//pcl::io::savePLYFile("plyFiles\\file" + std::to_string(j) + ".ply", v.point_cloud_list.set[j]);
//	cout << "*** Filtering started ***" << endl;
//	for (j = 0; j != v.point_cloud_list.set.size(); j++)
//		removeNaNFromPointCloud(v.point_cloud_list.set[j], v.point_cloud_list.set[j], mapping);
//	cout << "*** Filtering stopped ***" << endl;
//
//
//	//pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
//	//pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
//	//loadPLYFile("plyFiles\\file29.ply", *cloud);
//	//loadPLYFile("plyFiles\\file30.ply", *cloud2);
//
//	//PointCloudPtr p(new PointCloudT);
//	ICPAlgorithm icp_Alg;
//	//Matrix4f total_transformation;
//	//PointCloudPtr merge(new PointCloudT);
//	//*merge += *cloud;
//	//*merge += *cloud2;
//	//fp.save_point_cloud("before.ply", *merge);
//	//cout << "Start!!!" << endl;
//	//icp_Alg.aligning_two_pointcloud(*cloud, *cloud2, *p, 50, 0.1, total_transformation);
//	//cout << "finished!!!" << endl;
//	//cout << total_transformation << endl;
//
//	icp_Alg.register_with_result(v.point_cloud_list.set, 20, 0.05);
//	//icp_Alg.build_3d_map(v.point_cloud_list.set, 30, 0.09);
//	//icp_Alg.register_with_previous_aligned(v.point_cloud_list.set, 30, 0.1);
//	return 0;
//}
