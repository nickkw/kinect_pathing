#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_original (new pcl::PCLPointCloud2), cloud_filtered1 (new pcl::PCLPointCloud2),  cloud_filtered2 (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3 (new pcl::PointCloud<pcl::PointXYZ>), cloud_filtered4 (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_original);
  std::cerr << "PointCloud before filtering: " << cloud_original->width * cloud_original->height << " data points." << std::endl;

  // Create the intensity passthrough filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud_original);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (-0.1,0.1);
  pass.filter (*cloud_filtered1);

  // Create the z-axis passthrough filtering object
  pass.setInputCloud (cloud_filtered1);
  pass.setFilterFieldName ("y"); // Note that it is the 'y' axis in this case
  pass.setFilterLimits (0,230); // Set z-range here
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered1);

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
  proj.setInputCloud (cloud_filtered1);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_filtered2);

  // Create the radius outliers filtering object
  pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> outrem;
  outrem.setInputCloud(cloud_filtered2);
  outrem.setRadiusSearch(3.0f);
  outrem.setMinNeighborsInRadius (10);
  outrem.filter (*cloud_filtered1);

  // // Create the statistical outliers filtering object
  // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloud_blob);
  // sor.setMeanK (50);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*cloud_blob);

  // Convert to the templated PointCloud to template of PointXYZ type
  pcl::fromPCLPointCloud2 (*cloud_filtered1, *cloud_filtered3);

  // Find centroid and set as new origin (in preparation of scaling)
  Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
  Eigen::Matrix<float,4,1> centroid;
  pcl::compute3DCentroid(*cloud_filtered3, centroid); // Get centroid
  transform1.translation() << -centroid(0), -centroid(1), -centroid(2); // Shift origin to centroid
  // std::cerr << "Centroid: " << "x: " << centroid(0) << " y: " << centroid(1) << " z: " << centroid(2) << std::endl;
  pcl::transformPointCloud (*cloud_filtered3, *cloud_filtered3, transform1);

  // Scale the point cloud
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  float scale = 2.0;
  transform2.scale(scale);
  pcl::transformPointCloud (*cloud_filtered3, *cloud_filtered4, transform2);

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_filtered4);
  //chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_hull.pcd", *cloud_hull, false);
  writer.write ("table_scene_lms400_projection.pcd", *cloud_filtered3, false);

  return (0);
}