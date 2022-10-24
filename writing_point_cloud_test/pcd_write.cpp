#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main ()
 {
 // 创建点云的基本参数，以下指令描述了我们将要创建的模板化的PointCloud结构。每个点的类型都设置为pcl::PointXYZ，点云宽度为5，高度为1。
   pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.resize (cloud.width * cloud.height);
// 用随机生成的点向点云模板结构中填入数据
  for (auto& point: cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
// 将点云以ASCII码形式保存成PCD格式文件
  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
  std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

// 打印点云
  for (const auto& point: cloud)
    std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

  return (0);
}
