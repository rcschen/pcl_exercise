#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

int
main (int, char** argv)
{
  std::string filename = argv[1];
  std::cout << "Reading " << filename << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return -1;
  }

  std::cout << "points: " << cloud->points.size () << std::endl;

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (tree);

  // Output datasets
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

  // Use all neighbors in a sphere of radius 3cm
  normal_estimation.setRadiusSearch (0.03);

  // Compute the features
  normal_estimation.compute (*cloud_normals);

  // cloud_normals->points.size () should have the same size as the input cloud->points.size ()
  std::cout << "cloud_normals->points.size (): " << cloud_normals->points.size () << std::endl;
  return 0;
}

