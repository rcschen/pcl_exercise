#include <iostream>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

int
main (int argc, char** argv)
{
  if (argc < 2)
  {
    throw std::runtime_error ("Required arguments: filename.pcd");
  }

  std::string fileName = argv[1];
  std::cout << "Reading " << fileName << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file");
    return (-1);
  }

  std::cout << "Loaded " << cloud->points.size () << " points." << std::endl;

  // Compute the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud (cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

  normal_estimation.setRadiusSearch (0.03);

  normal_estimation.compute (*cloud_with_normals);
  // Setup the feature computation

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
  // Provide the original point cloud (without normals)
  fpfh_estimation.setInputCloud (cloud);
  // Provide the point cloud with normals
  fpfh_estimation.setInputNormals (cloud_with_normals);
  std::cout <<">>>>>>>>>>>>>ok"<<std::endl;

  // fpfhEstimation.setInputWithNormals(cloud, cloudWithNormals); PFHEstimation does not have this function
  // Use the same KdTree from the normal estimation
  fpfh_estimation.setSearchMethod (tree);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr pfh_features (new pcl::PointCloud<pcl::FPFHSignature33>);

  fpfh_estimation.setRadiusSearch (0.2);
  std::cout <<">>>>>>>>>>>>>ok?"<<std::endl;

  // Actually compute the spin images
  fpfh_estimation.compute (*pfh_features);

  std::cout << "output points.size (): " << pfh_features->points.size () << std::endl;

  // Display and retrieve the shape context descriptor vector for the 0th point.
  pcl::FPFHSignature33 descriptor = pfh_features->points[0];
  std::cout << descriptor << std::endl;

  return 0;
}
