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

#include <Eigen/Dense>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud1 (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCDReader reader;

  reader.read ("table_scene_lms400_downsampled.pcd", *cloud);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height << " data points." << std::endl;

  // Find centroid and set as new origin
  Eigen::Affine3f transform1 = Eigen::Affine3f::Identity();
  Eigen::Matrix<float,4,1> centroid;
  pcl::compute3DCentroid(*cloud, centroid); // Get centroid
  // transform1.translation() << -centroid(0)/2.0f, -centroid(1)/2.0f, -centroid(2)/2.0f; // Shift origin to centroid
  transform1.translation() << -centroid(0), -centroid(1), -centroid(2); // Shift origin to centroid
  std::cerr << "Centroid: " << "x: " << centroid(0) << " y: " << centroid(1) << " z: " << centroid(2) << std::endl;
  pcl::transformPointCloud (*cloud, *cloud1, transform1);

  // Expand the point cloud
  Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
  float scale = 2.0;
  transform2.scale(scale);
  pcl::transformPointCloud (*cloud1, *cloud, transform2);

  // Segmentation
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);
  std::cerr << "PointCloud after segmentation has: "
            << inliers->indices.size () << " inliers." << std::endl;

  // Project the model inliers
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setIndices (inliers);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients (coefficients);
  proj.filter (*cloud_projected);
  std::cerr << "PointCloud after projection has: "
            << cloud_projected->points.size () << " data points." << std::endl;

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (cloud_projected);
  //chull.setAlpha (0.1);
  chull.reconstruct (*cloud_hull);

  std::cerr << "Hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  pcl::PCDWriter writer;
  writer.write ("table_scene_lms400_hull.pcd", *cloud_hull, false);
  writer.write ("table_scene_lms400_hull2.pcd", *cloud1, false);

  return (0);
}