#include "pclviewer.h"
#include "ui_pclviewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  // Setup the cloud pointer
   cloud.reset (new PointCloudT);
  // The number of points in the cloud
  //cloud->points.resize (200);

  // The default color
  red   = 128;
  green = 128;
  blue  = 128;

//  // Fill the cloud with some points
//  for (size_t i = 0; i < cloud->points.size (); ++i)
//  {
//    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

//    cloud->points[i].r = red;
//    cloud->points[i].g = green;
//    cloud->points[i].b = blue;
//  }
 // pcl::io::loadPCDFile ("1.pcd", *cloud);
//  for (size_t i = 0; i < cloud->size(); i++)
//  {
////    cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
////    cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
////    cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
//      cloud->points[i].r = 255;
//      cloud->points[i].g = 255;
//      cloud->points[i].b = 255;
//  }

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

  // Connect R,G,B sliders and their functions
  connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (redSliderValueChanged (int)));
  connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (greenSliderValueChanged (int)));
  connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (blueSliderValueChanged (int)));
  connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased()));
  connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased()));
  connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  //Connect action
   connect (ui->action_2, SIGNAL(triggered()), this, SLOT (opencloudfile()));
   connect(ui->action_savefile,SIGNAL(triggered()),this,SLOT(savecloudfile()));
   connect(ui->actionReset,SIGNAL(triggered()),this,SLOT(resetScreen()));
   connect(ui->ButtonReconstruct,SIGNAL(clicked()),this,SLOT(passfilter()));
   connect(ui->actionDis_filter,SIGNAL(triggered()),this, SLOT(passfilter()));
   connect(ui->actionMm2m,SIGNAL(triggered()),this,SLOT(PointUnitMM2M()));
   connect(ui->action_PlanarSeg,SIGNAL(triggered()),this,SLOT(PlanarSegmennt()));
   connect(ui->action_OuterlierRevmoval,SIGNAL(triggered()),this,SLOT(StatisticalOulierRemoval()));
   connect(ui->actionVoxelGrid,SIGNAL(triggered()),this,SLOT(PCl_VoxelGrid()));
   connect(ui->action_EducCluster,SIGNAL(triggered()),this,SLOT(EuclideanClusterExtraction()));

   viewer->addPointCloud (cloud, "cloud");
   pSliderValueChanged (2);
   viewer->resetCamera ();
   ui->qvtkWidget->update ();


}

void
PCLViewer::randomButtonPressed ()
{
  printf ("Random button was pressed\n");

  // Set the new color
  for (size_t i = 0; i < cloud->size(); i++)
  {
//    cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
//    cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
//    cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
      //cloud->points[i].r = 255;
      //cloud->points[i].g = 255;
      //cloud->points[i].b = 255;
  }


  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::RGBsliderReleased ()
{
	//PointCloudT::Ptr cloud_filtered;
	//cloud_filtered.reset (new PointCloudT);
	//PointCloudT::Ptr cloud_filteredY;
	//cloud_filteredY.reset (new PointCloudT);
	//PointCloudT::Ptr cloud_filteredZ;
	//cloud_filteredZ.reset (new PointCloudT);
	//pcl::PassThrough<pcl::PointXYZ> pass;

	////x filter
	//pass.setInputCloud (currentcloud);
	//pass.setFilterFieldName ("x");
	//pass.setFilterLimits (-8, red);
	////pass.setFilterLimitsNegative (true);
	//pass.filter (*cloud_filtered);


	////y filter
	//pass.setInputCloud (cloud_filtered);
	//pass.setFilterFieldName ("y");
	//pass.setFilterLimits (-8, green);
	////pass.setFilterLimitsNegative (true);
	//pass.filter (*cloud_filteredY);



	////z filter
	//pass.setInputCloud (cloud_filtered);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (-8, blue);
	////pass.setFilterLimitsNegative (true);
	//pass.filter (*cloud_filteredZ);

	//cloud.reset (new PointCloudT);
	//cloud = cloud_filteredZ;

	//viewer->updatePointCloud (cloud, "cloud");
	//ui->qvtkWidget->update ();
}


void
PCLViewer::RXsliderReleased()
{
	//PointCloudT::Ptr cloud_filtered;
	//cloud_filtered.reset (new PointCloudT);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("x");
	//pass.setFilterLimits (0.0, red);
	////pass.setFilterLimitsNegative (true);
	//pass.filter (*cloud_filtered);

	//cloud.reset (new PointCloudT);
	//cloud = cloud_filtered;

	//viewer->updatePointCloud (cloud, "cloud");
	//ui->qvtkWidget->update ();
}

void 
PCLViewer::GYsliderReleased()
{
	//PointCloudT::Ptr cloud_filtered;
	//cloud_filtered.reset (new PointCloudT);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("y");
	//pass.setFilterLimits (0.0, green);
	////pass.setFilterLimitsNegative (true);
	//pass.filter (*cloud_filtered);

	//cloud.reset (new PointCloudT);
	//cloud = cloud_filtered;

	//viewer->updatePointCloud (cloud, "cloud");
	//ui->qvtkWidget->update ();
}

void
PCLViewer::BZsliderReleased()
{
	//PointCloudT::Ptr cloud_filtered;
	//cloud_filtered.reset (new PointCloudT);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud (cloud);
	//pass.setFilterFieldName ("z");
	//pass.setFilterLimits (0.0, blue);
	////pass.setFilterLimitsNegative (true);
	//pass.filter (*cloud_filtered);

	//cloud.reset (new PointCloudT);
	//cloud = cloud_filtered;

	//viewer->updatePointCloud (cloud, "cloud");
	//ui->qvtkWidget->update ();
}


void
PCLViewer::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
}

void
PCLViewer::redSliderValueChanged (int value)
{
  red = value;
  printf ("redSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::greenSliderValueChanged (int value)
{
  green = value;
  printf ("greenSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}

void
PCLViewer::blueSliderValueChanged (int value)
{
  blue = value;
  printf("blueSliderValueChanged: [%d|%d|%d]\n", red, green, blue);
}
void
PCLViewer::opencloudfile (void)
{    
	 cloud.reset (new PointCloudT);
	 currentcloud.reset(new PointCloudT);
	 tempcloud.reset (new PointCloudT);
	//创建文件打开窗口
	QString filename = QFileDialog::getOpenFileName(0,"Open pointcloud Files",QDir::currentPath(),"ALL point cloud files(*.*);;(*.pcd*.asc)");

	std::string strfilename = filename.toStdString();

	pcl::io::loadPCDFile (strfilename, *tempcloud);



	cloud = tempcloud;
	currentcloud = cloud;
	//viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
	//ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
	//viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
	//ui->qvtkWidget->update ();

	viewer->updatePointCloud (cloud, "cloud");
	ui->qvtkWidget->update ();
	viewer->resetCamera ();
	ui->qvtkWidget->update ();
	 
}
void
PCLViewer::savecloudfile(void)
{
	MyFileDialog myfiledialog(this,tr("Save point cloud file"));
	myfiledialog.setFilter("pcd file(*.pcd);;asc(*.asc)");
	myfiledialog.selectFile("Untitle");
	QString filename = myfiledialog.getFileNameAsSave();
	if ( !filename.isEmpty())
	{
		pcl::PCDWriter writer;
		writer.write<PointT> (filename.toStdString(), *cloud, false);
	}


}

void
PCLViewer::resetScreen()
{
	cloud.reset (new PointCloudT);
	currentcloud.reset(new PointCloudT);
	viewer->updatePointCloud (cloud, "cloud");
	ui->qvtkWidget->update ();
}
void
PCLViewer::StatisticalOulierRemoval()
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	PointCloudT::Ptr cloud_filtered (new PointCloudT);
	// 填入点云数据
	//pcl::PCDReader reader;
	// 把路径改为自己存放文件的路径
	//reader.read<pcl::PointXYZ> ("table_scene_lms400.pcd", *cloud);
	//std::cerr << "Cloud before filtering: " << std::endl;
	//std::cerr << *cloud << std::endl;
	// 创建滤波器对象
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (80);
	sor.setStddevMulThresh (0.005);
	sor.filter (*cloud_filtered);
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;
	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ> (".\\data\\inliers.pcd", *cloud_filtered, false);
	sor.setNegative (true);
	sor.filter (*cloud_filtered);
	writer.write<pcl::PointXYZ> (".\\data\\outliers.pcd", *cloud_filtered, false);
	
}
void
PCLViewer::PointUnitMM2M(VOID)
{
	for (size_t i = 0; i < cloud->size(); i++)
	{

		cloud->at(i).x = cloud->at(i).x/1000.0;
		cloud->at(i).y = cloud->at(i).y/1000.0;
		cloud->at(i).z = cloud->at(i).z/1000.0;
	}

	
	currentcloud = cloud;


	viewer->updatePointCloud (cloud, "cloud");
	ui->qvtkWidget->update ();
	viewer->resetCamera ();
	ui->qvtkWidget->update ();

}
void
PCLViewer::PlanarSegmennt()
{   
	// All the objects needed
	pcl::PCDReader reader;
	pcl::PassThrough<PointT> pass;
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg; 
	pcl::PCDWriter writer1;
	pcl::ExtractIndices<PointT> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());



	// Datasets
	pcl::PointCloud<PointT>::Ptr cloud1 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<PointT>::Ptr cloud_filtered2 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

	// Estimate point normals
	ne.setSearchMethod (tree);
	ne.setInputCloud (cloud);
	ne.setKSearch (50);
	ne.compute (*cloud_normals);

	// Create the segmentation object for the planar model and set all the parameters
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight (0.1);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.01);
	seg.setInputCloud (cloud);
	seg.setInputNormals (cloud_normals);
	// Obtain the plane inliers and coefficients
	seg.segment (*inliers_plane, *coefficients_plane);
	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from th e input cloud
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);

	// Write the planar inliers to disk
	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
	extract.filter (*cloud_plane);
	//std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
	
	writer1.write ("plane.pcd", *cloud_plane, false);

	// Remove the planar inliers, extract the rest
	extract.setNegative (true);
	extract.filter (*cloud_filtered2);


	writer1.write ("Withoutplane.pcd", *cloud_filtered2, false);
	cloud = cloud_filtered2;
	currentcloud = cloud;

	viewer->updatePointCloud (cloud, "cloud");
	ui->qvtkWidget->update ();
	viewer->resetCamera ();
	ui->qvtkWidget->update ();


	//extract_normals.setNegative (true);
	//extract_normals.setInputCloud (cloud_normals);
	//extract_normals.setIndices (inliers_plane);
	//extract_normals.filter (*cloud_normals2);

}

void
PCLViewer::PCl_VoxelGrid(void)
{
	//pcl::PointCloud<PointT>::Ptr cloud (new PointCloudT);
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new PointCloudT); 


	//sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
	// 填入点云数据
	//pcl::PCDReader reader;
	// 把路径改为自己存放文件的路径
	//reader.read ("table_scene_lms400.pcd", *cloud); // 记住要事先下载这个数据集！
	std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
		<< " data points (" << pcl::getFieldsList (*cloud) << ").";
	// 创建滤波器对象
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.003f, 0.003f, 0.003f);
	sor.filter (*cloud_filtered);
	std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
		<< " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";
	pcl::PCDWriter writer;
	writer.write (".\\data\\VoxelGrid.pcd", *cloud_filtered, false);

	cloud = cloud_filtered;
	currentcloud = cloud;

	viewer->updatePointCloud (cloud, "cloud");
	ui->qvtkWidget->update ();
	viewer->resetCamera ();
	ui->qvtkWidget->update ();
}

void 
PCLViewer::EuclideanClusterExtraction()
{   
	
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.004); // 4mm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (50000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

	int j = 0;
	
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  
}
void
PCLViewer::passfilter()
{
	PointCloudT::Ptr cloud_filtered;
	cloud_filtered.reset (new PointCloudT);
	PointCloudT::Ptr cloud_filteredY;
	cloud_filteredY.reset (new PointCloudT);
	PointCloudT::Ptr cloud_filteredZ;
	cloud_filteredZ.reset (new PointCloudT);
	pcl::PassThrough<PointT> pass;

	//x filter
	pass.setInputCloud (currentcloud);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (ui->SpinBox_MIN_X->value(), ui->SpinBox_MAX_X->value());
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);


	//y filter
	pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (ui->SpinBox_MIN_Y->value(), ui->SpinBox_MAX_Y->value());
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filteredY);



	//z filter
	pass.setInputCloud (cloud_filteredY);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (ui->SpinBox_MIN_Z->value(), ui->SpinBox_MAX_Z->value());
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filteredZ);

	cloud.reset (new PointCloudT);
	cloud = cloud_filteredZ;

	currentcloud.reset(new PointCloudT);
	currentcloud = cloud;

	viewer->updatePointCloud (cloud, "cloud");
	viewer->resetCamera ();
	ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
