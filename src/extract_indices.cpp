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

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2),  cloud_filtered_blob2 (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read ("table_scene_lms400.pcd", *cloud_blob);

  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // Create the passthrough filtering object
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud (cloud_blob);
  pass.setFilterFieldName ("intensity");
  pass.setFilterLimits (-0.1,0.1);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_blob);

  // Create the passthrough filtering object
  pass.setInputCloud (cloud_blob);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (250,330);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_blob);

  // std::cerr << "Cloud after passthrough filtering: " << std::endl;
  // for (size_t i = 0; i < cloud_filtered_blob->points.size (); ++i)
  // std::cerr << "    " << cloud_filtered_blob->points[i].x << " " 
  //                       << cloud_filtered_blob->points[i].y << " " 
  //                       << cloud_filtered_blob->points[i].z << std::endl;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after downsample filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  // seg.setAxis()
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.50);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  return (0);
}
