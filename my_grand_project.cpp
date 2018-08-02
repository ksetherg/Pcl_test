#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int create_cloud (int, int);
bool load_cloud (const std::string &, pcl::PointCloud<pcl::PointXYZ> &);
bool save_cloud (const std::string &, const pcl::PointCloud<pcl::PointXYZ> &);

//////////////////////////////////
int
create_cloud (int w, int h)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width    = w;
    cloud.height   = h;
    cloud.is_dense = false;
    cloud.points.resize (cloud.width * cloud.height);

    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
   }

    if (!save_cloud ("test_pcd.pcd", cloud))
        return (-1);

    return 0;
}

bool
load_cloud (const std::string &filename, pcl::PointCloud<pcl::PointXYZ> &cloud)
{

    if (pcl::io::loadPCDFile (filename, cloud) < 0)
        return (false);
    std::cerr << "width: "
              << cloud.width
              << " height: "
              << cloud.height
              << std::endl;

    return (true);
    
}

bool
save_cloud(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ> &output)
{
    if (pcl::io::savePCDFileASCII (filename, output) < 0)
        return (false);
    std::cerr << "Saved " 
              << output.points.size () 
              << " data points to test_pcd.pcd" 
              << std::endl;
    return (true);          
}
int
main()
{
    int w = 3;
    int h = 1;
    create_cloud(w, h);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (!load_cloud ("test_pcd.pcd", *cloud))
        return (-1);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z << std::endl;
    
    
    return 0;    
}