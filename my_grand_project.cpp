#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>
#include <vector>
#include <iterator>
#include <random>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/thread/thread.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/gregorian/gregorian_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/thread/thread.hpp>
#include <math.h>

int create_cloud (int, int);
bool load_cloud (const std::string &, pcl::PointCloud<pcl::PointXYZ> &);
bool save_cloud (const std::string &, const pcl::PointCloud<pcl::PointXYZ> &);
void add_gaussian_noise(pcl::PointCloud<pcl::PointXYZ> &, pcl::PointCloud<pcl::PointXYZ> &, float, float);
double L2norm(pcl::PointXYZ &, pcl::PointXYZ &);
double compute_metric_abs(pcl::PointCloud<pcl::PointXYZ> &);
double compute_metric_mean(pcl::PointCloud<pcl::PointXYZ> &);

const pcl::PointXYZ obs = pcl::PointXYZ(0., 0., 0.); //наблюдатель

int
create_cloud (int w, int h)
{
    //создаем свое облако точек
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
    //подгружаем облако, если можем, если смогли, то выводим его параметры
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
    //сохраняем облако точек, если получается, то выводим сколько точек в облаке сохранилось
    if (pcl::io::savePCDFileASCII (filename, output) < 0)
        return (false);
    std::cerr << "Saved " 
              << output.points.size () 
              << " data points to test_pcd.pcd" 
              << std::endl;
    return (true);          
}

void 
add_gaussian_noise( pcl::PointCloud<pcl::PointXYZ> &input,
                    pcl::PointCloud<pcl::PointXYZ> &noized_cloud,
                    float mean, float disp)
{   
    //просто добавляем шумм из нормального распределения с матожиданием mean и дисперсией disp
    noized_cloud.points.resize(input.points.size());
    noized_cloud.width = input.width;
    noized_cloud.height = input.height;
/* 
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean, disp);

    for (size_t point_i = 0; point_i < input.points.size (); ++point_i)
    {
        noized_cloud->points[point_i].x = input.points[point_i].x + static_cast<float>(dist(generator));
        noized_cloud->points[point_i].y = input.points[point_i].y + static_cast<float>(dist(generator));
        noized_cloud->points[point_i].z = input.points[point_i].z + static_cast<float>(dist(generator));
    }
*/
    boost::mt19937 rng; 
    rng.seed (static_cast<unsigned int> (time (0)));
    boost::normal_distribution<> nd (mean, disp);
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

    for (size_t point_i = 0; point_i < input.points.size (); ++point_i)
    {
        noized_cloud.points[point_i].x = input.points[point_i].x + static_cast<float> (var_nor ());
        noized_cloud.points[point_i].y = input.points[point_i].y + static_cast<float> (var_nor ());
        noized_cloud.points[point_i].z = input.points[point_i].z + static_cast<float> (var_nor ());
    }
}

double
L2norm(pcl::PointXYZ &x, const pcl::PointXYZ &y)
{   
    //обычная евклидова норма 
    double dist;
    dist = sqrt(pow(x.x - y.x, 2) + pow(x.y - y.y, 2) + pow(x.z - y.z, 2));
    return dist;
}

double
compute_metric_abs(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // абсолютная метрика для облака
    double sum = 0;
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        sum += L2norm(cloud.points[i], obs);
    }
    return sum;
}

double
compute_metric_mean(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    // средняя метрика для облака
    double sum = 0;
    size_t n = cloud.points.size ();
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
        sum += L2norm(cloud.points[i], obs);
    }
    return sum/n;
}

int
main(int argc, char **argv)
{
    int w = 10;
    int h = 1;
    float mean = 0; //матожидание
    float disp = 0.1; //дисперсия

    //create_cloud(w, h); //создаем облако точек с необходимым количесвтом точек и структурой
                          //этот метод создавался для того, чтобы тестить программу на малых количествах точек, для удобства

    if (argc != 3)
    {
        std::cerr << "must be one input PCD file and one output PCD file to continue" << std::endl;
        return -1;
    }
    
    //загружаем в cloud наше исходное облако

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (!load_cloud (argv[1], *cloud))
        return (-1);

    for (size_t i = 0; i < cloud->points.size (); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " "    << cloud->points[i].y
                  << " "    << cloud->points[i].z << std::endl;

    //добавляем к облаку шум 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mod (new pcl::PointCloud<pcl::PointXYZ>);
    add_gaussian_noise(*cloud, *cloud_mod, mean, disp);
    save_cloud (argv[2], *cloud_mod);
    /*
    //загружаем в noized_cloud новое зашумленное облако
    //вообще не очень понятно зачем сохранять и снова загружать модифицированное облако, 
    //когда удобно работать сразу внутри программы

    pcl::PointCloud<pcl::PointXYZ>::Ptr noized_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (!load_cloud ("test_pcd_noized.pcd", *noized_cloud))
        return (-1);

    for (size_t i = 0; i < noized_cloud->points.size (); ++i)
        std::cout << "    " << noized_cloud->points[i].x
                  << " "    << noized_cloud->points[i].y
                  << " "    << noized_cloud->points[i].z << std::endl;
    */
    
    //считаем и выводим метрику
    double abs_m; //абсолютная разница 
    double mean_m; //средняя разница 
    abs_m = abs(compute_metric_abs(*cloud_mod) - compute_metric_abs(*cloud));
    mean_m = abs(compute_metric_mean(*cloud_mod) - compute_metric_mean(*cloud));
    std::cout << "абсолютная разница: " << abs_m 
              << std::endl
              << "средняя разница: " << mean_m
              <<std::endl;
    return 0;    
}