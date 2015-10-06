#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
             std::string fileName = argv[1];
             std::cout << "Reading " << fileName << std::endl;

             // load point cloud
             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
             pcl::io::loadPCDFile (fileName, *cloud);

             // estimate normals
             pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

             pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
             //ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
             //ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
             ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);


             //ne.setMaxDepthChangeFactor(12.0f);
             //ne.setNormalSmoothingSize(100.1f);
             ne.setInputCloud(cloud);
             ne.compute(*normals);

             // visualize normals
             pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
             //pcl::visualization::PCLVisualizer viewer("PCL Viewer");
             //viewer.setBackgroundColor (0.0, 0.0, 0.5);
             //viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

             while (!viewer.wasStopped ())
             {
               //viewer.spinOnce ();
               viewer.showCloud (cloud);
             }
             return 0;
}
