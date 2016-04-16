//#include <iostream>
//#include <string>
//
//#include <pcl/console/time.h>   // TicToc 
//#include <pcl/console/parse.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/visualization/pcl_visualizer.h>
//
//typedef pcl::PointXYZRGBA PointT;
//typedef pcl::PointCloud<PointT> PointCloudT;
//
//using std::cout;
//using std::endl;
//
//pcl::PolygonMesh::Ptr scan_mesh;
//pcl::PolygonMesh::Ptr cad_mesh;
//PointCloudT::Ptr scan_cloud;
//PointCloudT::Ptr cad_cloud;
//bool update_models = false;
//int icp_iterations(1);
//double voxel_grid_size(0.0005);  // 0.5 mm 
//double max_corr_distance(0.001);  // 1 mm 
//int iterations_count(0);
//
//void
//keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
//	void* nothing)
//{
//	static int total_icp_time(0);
//
//	if (event.getKeySym() == "space" && event.keyDown())
//	{
//		if (iterations_count % 100 == 0 && iterations_count != 0)
//		{
//			max_corr_distance /= 3.0;
//			cout << "Maximum correspondence distance = " << max_corr_distance << endl;
//		}
//
//		// Use ICP to align point clouds 
//		pcl::IterativeClosestPoint<PointT, PointT> icp;
//		icp.setMaximumIterations(icp_iterations);
//		icp.setInputSource(cad_cloud);
//		icp.setInputTarget(scan_cloud);
//		icp.setMaxCorrespondenceDistance(max_corr_distance);  // 1/5th of the object size 
//
//															  // The user pressed "space" 
//		pcl::console::TicToc time;
//		time.tic();
//		icp.align(*cad_cloud);
//		double icp_time = time.toc();
//		total_icp_time += icp_time;
//		cout << "Applied " << icp_iterations << " ICP iteration in " << icp_time << " ms (total = " << total_icp_time << " ms)" << endl;
//
//		if (icp.hasConverged())
//		{
//			iterations_count += icp_iterations;
//			cout << "ICP has converged, score is " << icp.getFitnessScore() << endl;
//			cout << "ICP iterations = " << iterations_count << endl << endl;
//
//			PointCloudT::Ptr cloud(new PointCloudT);
//			pcl::fromPCLPointCloud2(cad_mesh->cloud, *cloud);
//			pcl::transformPointCloud(*cloud, *cloud, icp.getFinalTransformation());
//			pcl::toPCLPointCloud2(*cloud, cad_mesh->cloud);
//			update_models = true;
//		}
//		else
//		{
//			PCL_ERROR("\nICP has not converged.\n");
//			//return (-1); 
//		}
//	}
//}
//
//void
//displayHelp(int argc,
//	char** argv)
//{
//	PCL_INFO("\nUsage: %s [OPTION] DEST SOURCE_MESH SOURCE_CLOUD\n", argv[0]);
//	PCL_INFO("Align DEST point cloud / mesh to DEST point cloud using ICP.\n\n");
//
//	PCL_INFO("DEST should be a mesh, the point cloud is extracted for the alignment.\n");
//	PCL_INFO("SOURCE_MESH is the mesh (usually a CAD) to be aligned,\nSOURCE_CLOUD should be an uniformly sampled cloud extracted from SOURCE_MESH.\n\n");
//
//	PCL_INFO("SOURCE_MESH, SOURCE_CLOUD and DEST must be in the PLY (Polygon File Format) file format.\n");
//	PCL_INFO("To trigger the alignment, press \"space\" when the focus is on the visualizer window.\n\n");
//
//	PCL_INFO("Available options:\n"
//		"\t-i, --iterations           Specify the number of ICP iterations each time \"space\" is pressed. Default is 1\n"
//		"\t-vgs --voxel_grid_size     The size of the voxel grid (in meters) used to down-sample data for faster alignment. Use -1 to disable the voxel grid. Default is 0.0005 meters\n"
//		"\t-mcd --max_corr_distance  The maximum correspondence distance (in meters) for the ICP alignment. Default is 0.001 meters\n\n");
//}
//
//int
//main(int argc,
//	char* argv[])
//{
//	// Checking program arguments 
//	std::vector<int> ply_file_indices = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
//
//	if (argc < 3 || ply_file_indices.size() < 2)
//	{
//		displayHelp(argc, argv);
//		return (-1);
//	}
//
//	cout << "Loading input data" << endl;
//	cout << argv[ply_file_indices[0]] << endl;
//	cout << argv[ply_file_indices[1]] << endl;
//	// Loading scan mesh 
//	scan_mesh.reset(new pcl::PolygonMesh);
//	//pcl::PLYReader::read(argv[ply_file_indices[0]], *scan_cloud, 0);
//	//pcl::PLYReader::read()
//
//
//
//	if (pcl::io::loadPolygonFilePLY(argv[ply_file_indices[0]], *scan_mesh) == 0)
//	{
//		PCL_ERROR("Failed to read PLY file %s\n", argv[ply_file_indices[0]]);
//		return (-1);
//	}
//	cout << "PLY 1 LOADED" << endl;
//
//	// Loading CAD mesh 
//	cad_mesh.reset(new pcl::PolygonMesh);
//	if (pcl::io::loadPolygonFilePLY(argv[ply_file_indices[1]], *cad_mesh) == 0)
//	{
//		PCL_ERROR("Failed to read PLY file %s\n", argv[ply_file_indices[1]]);
//		return (-1);
//	}
//
//	cout << "PLY 2 LOADED" << endl;
//
//	cad_cloud.reset(new PointCloudT);  // CAD point cloud 
//	if (ply_file_indices.size() > 2)
//	{
//		if (pcl::io::loadPLYFile(argv[ply_file_indices[2]], *cad_cloud) != 0)
//		{
//			PCL_ERROR("Could not load file %s.\n", argv[ply_file_indices[2]]);
//			return (-1);
//		}
//	}
//	else
//	{
//		// Convert CAD mesh to CAD cloud 
//		pcl::fromPCLPointCloud2(cad_mesh->cloud, *cad_cloud);
//	}
//
//	if (argc > 3)
//	{
//		pcl::console::parse_argument(argc, argv, "--iterations", icp_iterations);
//		pcl::console::parse_argument(argc, argv, "-i", icp_iterations);
//		if (icp_iterations < 1)
//		{
//			PCL_WARN("Number of maximum iterations must be >= 1\n");
//			icp_iterations = 1;
//		}
//
//		pcl::console::parse_argument(argc, argv, "--voxel_grid_size", voxel_grid_size);
//		pcl::console::parse_argument(argc, argv, "-vgs", voxel_grid_size);
//
//		pcl::console::parse_argument(argc, argv, "--max_corr_distance", max_corr_distance);
//		pcl::console::parse_argument(argc, argv, "-mcd", max_corr_distance);
//		max_corr_distance = fabs(max_corr_distance);
//	}
//
//	scan_cloud.reset(new PointCloudT);  // Scan point cloud 
//	pcl::fromPCLPointCloud2(scan_mesh->cloud, *scan_cloud);
//
//	pcl::io::loadPLYFile(argv[ply_file_indices[0]], *scan_cloud);
//	pcl::io::loadPLYFile(argv[ply_file_indices[1]], *cad_cloud);
//
//	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
//
//	// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//	float theta = M_PI / 4; // The angle of rotation in radians
//	transform_1(0, 0) = cos(theta);
//	transform_1(0, 0.5) = -sin(theta);
//	transform_1(0.5, 0) = sin(theta);
//	transform_1(0.5, 0.5) = cos(theta);
//	//    (row, column)
//
//	// Define a translation of 2.5 meters on the x axis.
//	transform_1(0, 3) = 0.5;
//
//	// Print the transformation
//	printf("Method #1: using a Matrix4f\n");
//	std::cout << transform_1 << std::endl;
//	// Executing the transformation
//	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
//	// You can either apply transform_1 or transform_2; they are the same
//	pcl::transformPointCloud(*cad_cloud, *cad_cloud, transform_1);
//
//	cout << "Parameters" << endl << "----------" << endl;
//	cout << "ICP iterations steps: " << icp_iterations << endl;
//	cout << "Voxel grid size: " << voxel_grid_size << endl;
//	cout << "Maximum correspondence distance: " << max_corr_distance << endl;
//	cout << "Scan cloud size = " << scan_cloud->size() << endl;
//	cout << "CAD cloud size = " << cad_cloud->size() << endl << endl;
//
//	if (voxel_grid_size > 0)
//	{
//		// Down-sample the data for faster alignment 
//		cout << "Filtering scan and CAD clouds" << endl << "----------------------------" << endl;
//		pcl::console::TicToc time;
//		time.tic();
//		pcl::VoxelGrid<PointT> voxel_grid;
//		voxel_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
//		voxel_grid.setInputCloud(scan_cloud);
//		voxel_grid.filter(*scan_cloud);
//		voxel_grid.setInputCloud(cad_cloud);
//		voxel_grid.filter(*cad_cloud);
//		cout << "Filtered 2 clouds in " << time.toc() << " ms " << endl;
//		cout << "Scan cloud size: " << scan_cloud->size() << endl;
//		cout << "CAD cloud size: " << cad_cloud->size() << endl << endl;
//	}
//
//	// Visualizer 
//	pcl::visualization::PCLVisualizer viewer;
//	int v1(0), v2(1);
//	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
//	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
//
//	viewer.addPolygonMesh(*cad_mesh, "mesh_cad", v1);
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255 / 255., 50 / 255., 50 / 255., "mesh_cad", v1);
//	viewer.addPolygonMesh(*scan_mesh, "mesh_scan", v1);
//
//	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cad_color(cad_cloud, 255, 50, 50);
//	viewer.addPointCloud(cad_cloud, cloud_cad_color, "cloud_cad", v2);
//	viewer.addPointCloud(scan_cloud, "cloud_scan", v2);
//
//	// Display the visualizer 
//	viewer.setCameraPosition(1.42379, 0.159511, -0.179308, 0.217544, 0.42156, 0.880319);
//	viewer.resetCamera();
//	viewer.addText("0 iterations", 0, 20, 16, 1, 1, 1, "icp_iterations", 0);
//	while (!viewer.wasStopped())
//	{
//		if (update_models)
//		{
//			viewer.updateText(boost::lexical_cast<std::string> (iterations_count) + " iterations", 0, 20, "icp_iterations");
//			viewer.updatePolygonMesh(*scan_mesh, "mesh_scan");
//			viewer.updatePolygonMesh(*cad_mesh, "mesh_cad");
//			viewer.updatePointCloud(scan_cloud, "cloud_scan");
//			pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_cad_color(cad_cloud, 255, 50, 50);
//			viewer.updatePointCloud(cad_cloud, cloud_cad_color, "cloud_cad");
//			update_models = false;
//		}
//		viewer.spinOnce();
//	}
//	return (0);
//}