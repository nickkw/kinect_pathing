#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/filters/radius_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2),  cloud_filtered_blob2 (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the intensity passthrough filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud_blob);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (-0.1,0.1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_blob);

  // Create the z-axis passthrough filtering object
  pass.setInputCloud (cloud_blob);
  pass.setFilterFieldName ("y"); // Note that it is the 'y' axis in this case
  pass.setFilterLimits (0,230); // Set z-range here
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_blob);


  // Create the inliers filtering object
  // Create a set of planar coefficients to project on the xy plane
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = 0;
  coefficients->values[1] = 1.0;
  coefficients->values[2] = 0;
  coefficients->values[3] = 0;

  pcl::ProjectInliers<pcl::PCLPointCloud2> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud_blob);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_filtered_blob2);

  // Create the radius outliers filtering object
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
  // build the filter
  outrem.setInputCloud(cloud_filtered_blob2);
  outrem.setRadiusSearch(3.0f);
  outrem.setMinNeighborsInRadius (10);
  // apply filter
  outrem.filter (*cloud_filtered_blob);

  // // Create the statistical outliers filtering object
  // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud_blob);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*cloud_blob);


  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  return (0);
}
