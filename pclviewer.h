#ifndef PCLVIEWER_H
#define PCLVIEWER_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "MyQtClass.h"
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

//typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public slots:
  void
  randomButtonPressed ();

  void
  RGBsliderReleased ();

  void
  RXsliderReleased ();

  void
  GYsliderReleased ();

  void
  BZsliderReleased ();


  void
  pSliderValueChanged (int value);

  void
  redSliderValueChanged (int value);

  void
  greenSliderValueChanged (int value);

  void
  blueSliderValueChanged (int value);

  //打开点云文件
  void 
  opencloudfile(void);
  void 
  savecloudfile(void);
  void 
  resetScreen(void);

  void 
  passfilter();
  
  void 
  StatisticalOulierRemoval(void);

  void 
  PointUnitMM2M(void);

  void
  PlanarSegmennt(void);

  void 
  PCl_VoxelGrid(void);

  //欧氏聚类
  void 
  EuclideanClusterExtraction(void);


protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;
  PointCloudT::Ptr currentcloud;
  PointCloudT::Ptr tempcloud;

  unsigned int red;
  unsigned int green;
  unsigned int blue;

private:
  Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H
