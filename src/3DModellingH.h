#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/ply/ply.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <stdio.h>
#include <sstream>
#include <stdlib.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> MyPointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef const pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::PCLPointCloud2ConstPtr PointCloudConstPtr2;