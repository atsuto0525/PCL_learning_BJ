#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int
main ()
{
  // [1]首先我们使用以下语句创建一个指向pcl::PointXYZ类型的共享指针cloud，此处pcl::PointXYZ类型指的是只有XYZ三个维度位置信息的点云类型。
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  // [2]从磁盘加载PointCloud数据(例如文件名为test_pcd.pcd的文件)，未成功读取则返回-1
  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

  // [3]打印pcd文件中点云的数据量（宽度*高度）；
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

  // [4]打印出文件中点的位置信息，以下方式常用，需熟练使用。
  for (const auto& point: *cloud)
    std::cout << " " << point.x
              << " "    << point.y
              << " "    << point.z << std::endl;

  return (0);
}
