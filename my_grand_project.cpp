#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <vector>
#include <iterator>
#include <random>

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

void //возможно стоит выводить облако, оставлю оут пока что pcl::PointCloud<pcl::PointXYZ> &output,
add_gaussian_noise(pcl::PointCloud<pcl::PointXYZ> &input,
                    
                    float mean,
                    float disp)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr noized_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    noized_cloud->points.resize(input.points.size());
    noized_cloud->width = input.width;
    noized_cloud->height = input.height;

    std::vector<double> data;

    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, disp);

    for (size_t point_i = 0; point_i < input.points.size (); ++point_i)
    {
        noized_cloud->points[point_i].x = input.points[point_i].x + static_cast<float>(dist(generator));
        noized_cloud->points[point_i].y = input.points[point_i].y + static_cast<float>(dist(generator));
        noized_cloud->points[point_i].z = input.points[point_i].z + static_cast<float>(dist(generator));
        data.insert(data.end(), dist(generator));
    }

    std::copy( data.begin(),   
          data.end(),     
          std::ostream_iterator<double>(std::cout," ") 
        );

    save_cloud ("noized_cloud.pcd", *noized_cloud);
      
    /*
    boost::mt19937 rng; 
    rng.seed (static_cast<unsigned int> (time (0)));
    boost::normal_distribution<> nd (0, disp);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

    for (size_t point_i = 0; point_i < input.points.size (); ++point_i)
    {
        noized_cloud->points[point_i].x = input.points[point_i].x + static_cast<float> (var_nor ());
        noized_cloud->points[point_i].y = input.points[point_i].y + static_cast<float> (var_nor ());
        noized_cloud->points[point_i].z = input.points[point_i].z + static_cast<float> (var_nor ());
    }
    */

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
    

    add_gaussian_noise (*cloud, 0., 1.);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mod (new pcl::PointCloud<pcl::PointXYZ>);
    
    if (!load_cloud ("noized_cloud.pcd", *cloud_mod))
        return (-1);

    for (size_t i = 0; i < cloud_mod->points.size (); ++i)
    std::cout << "    " << cloud_mod->points[i].x
              << " "    << cloud_mod->points[i].y
              << " "    << cloud_mod->points[i].z << std::endl;
    
    
    return 0;    
}