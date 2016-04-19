#include "3DModellingH.h"
#include "FileProcessingH.h"
#include "ICPAlgorithm.h"
class PointCloudSet
{
public:
	vector<PointCloudT> set;
	vector<PointCloudT>::iterator it;

	PointCloudSet();

	~PointCloudSet();

private:

};

PointCloudSet::PointCloudSet()
{
	this->it = set.begin();
}
PointCloudSet::~PointCloudSet()
{
}

class SimpleOpenNIViewer
{
public:
	pcl::visualization::CloudViewer viewer;
	pcl::Grabber* interface;
	FileProcessing file;
	ICPAlgorithm icp_Alg;
	PointCloudSet point_cloud_list;
	int frameNumber;
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
		this->frameNumber = 0;
	}
	~SimpleOpenNIViewer()
	{

	}
	void cloud_cb_(PointCloudConstPtr &cloud)
	{

		PointCloudPtr cloud_out(new PointCloudT);
		std::vector<int> mapping;
		PointCloudPtr cloud_in(new PointCloudT);
		copyPointCloud(*cloud, *cloud_in);
		pcl::removeNaNFromPointCloud(*cloud_in, *cloud_out, mapping);
		this->point_cloud_list.it = this->point_cloud_list.set.insert(this->point_cloud_list.it, *cloud_out);
		//pcl::io::savePLYFile("file" + std::to_string(frameNumber++) + ".ply", *cloud_out);
		frameNumber++;
		if (!viewer.wasStopped())
			viewer.showCloud(cloud_out);
	}

	void run()
	{
		boost::function<void(PointCloudConstPtr&)> f =
			boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

		this->interface->registerCallback(f);

		this->interface->start();

		while (!viewer.wasStopped())
			boost::this_thread::sleep(boost::posix_time::seconds(1));

		this->interface->stop();
	}

};

int main()
{
	//SimpleOpenNIViewer v;
	//v.run();

	vector<PointCloudT>::size_type j;
	PCLPointCloud2 pcl2;
	bool format = true;
	bool use_camera = true;

	//v.file.makePLYFromPointCloudSet("PointCloud", pcl2, format, use_camera, &v.point_cloud_list.set);

	//for (j = 0; j != v.point_cloud_list.set.size(); j++)
	//pcl::io::savePLYFile("plyFiles\\file" + std::to_string(j) + ".ply", v.point_cloud_list.set[j]);



	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	loadPLYFile("plyFiles\\file29.ply", *cloud);
	loadPLYFile("plyFiles\\file30.ply", *cloud2);

	PointCloudPtr p(new PointCloudT);
	ICPAlgorithm icp_Alg;
	PointCloudPtr merge(new PointCloudT);
	*merge += *cloud;
	*merge += *cloud2;
	icp_Alg.save_point_cloud("before.ply", *merge);
	cout << "Start!!!" << endl;
	icp_Alg.aligning_two_pointcloud(*cloud, *cloud2, *p, 10, 0.0001);
	cout << "finished!!!" << endl;

	//v.icp_Alg.Register(v.point_cloud_list.set,v.frameNumber);
	return 0;
}