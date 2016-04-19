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
	//pcl::io::savePLYFile("file" + std::to_string(j) + ".ply", v.point_cloud_list.set[j]);
	srand(time(NULL));

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT>);
	// Generate pointcloud data
	cloud->width = 5;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	cloud2->width = 5;
	cloud2->height = 1;
	cloud2->points.resize(cloud2->width * cloud2->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = (1024* rand() / (RAND_MAX + 1))%30;
		cloud->points[i].y = (1024 * rand() / (RAND_MAX + 1)) % 30;
		cloud->points[i].z = (1024 * rand() / (RAND_MAX + 1)) % 30;
		cloud->points[i].rgba = 0;
		cloud2->points[i].x = (1024 * rand() / (RAND_MAX + 1)) % 30;
		cloud2->points[i].y = (1024 * rand() / (RAND_MAX + 1)) % 30;
		cloud2->points[i].z = (1024 * rand() / (RAND_MAX + 1)) % 30;
		cloud2->points[i].rgba = 0;
	}

	PointCloudPtr p(new PointCloudT);
	ICPAlgorithm icp_Alg;
	
		
		cout << "start icp:" << endl;
		for (int i = 0; i < cloud->points.size(); i++)
		{
			cout << cloud->points[i] << endl;
		}
		for (int i = 0; i < cloud2->points.size(); i++)
		{
			cout << cloud2->points[i] << endl;
		}
		
		icp_Alg.aligning_two_pointcloud(*cloud, *cloud2, *p, 10, 1);
		cout << "finished!!!" << endl;
		for (int i = 0; i < p->points.size(); i++)
		{
			cout << p->points[i] << endl;
		}
		

	
	//v.icp_Alg.Register(v.point_cloud_list.set,v.frameNumber);
	return 0;
}