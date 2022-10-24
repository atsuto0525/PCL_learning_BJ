 #include <pcl/point_cloud.h>
 #include <pcl/octree/octree_search.h>
 
 #include <iostream>
 #include <vector>
 #include <ctime>
 
 int
 main ()
 {
   srand ((unsigned int) time (NULL));
  // [1]创建点云指针
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
 
   // [2] 构造1000个随机的点云数据
   cloud->width = 1000;
   cloud->height = 1;
   cloud->points.resize (cloud->width * cloud->height);
 
   for (std::size_t i = 0; i < cloud->size (); ++i)
   {
     (*cloud)[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
     (*cloud)[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
     (*cloud)[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
   }
 
   float resolution = 128.0f;
  // [3]八叉树点云搜索实例
   pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (resolution);
  // [4]将点云设置成八叉树结构
   octree.setInputCloud (cloud);
   octree.addPointsFromInputCloud ();
  // [5]随机定义一个要查找的点
   pcl::PointXYZ searchPoint;
 
   searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
   searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
   searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);
 
  //**************************体素搜索（搜索相同体素内的点）************************
  // [6]创建点云索引容器
   std::vector<int> pointIdxVec;
  // [7]开始进行体素内近邻点搜索
  if (octree.voxelSearch (searchPoint, pointIdxVec))
  {
  // [8] 将查找点所在体素内的邻点全都输出打印到命令窗口
     std::cout << "Neighbors within voxel search at (" << searchPoint.x 
      << " " << searchPoint.y 
      << " " << searchPoint.z << ")" 
      << std::endl;
               
     for (std::size_t i = 0; i < pointIdxVec.size (); ++i)
    std::cout << "    " << (*cloud)[pointIdxVec[i]].x 
        << " " << (*cloud)[pointIdxVec[i]].y 
        << " " << (*cloud)[pointIdxVec[i]].z << std::endl;
   }
 
     //*************************K近邻搜索（搜索K个最近点）************************
   int K = 10;
  // 创建点索引容器，用于按距离保存搜索到的点
   std::vector<int> pointIdxNKNSearch;
    // 创建点距离容器 
   std::vector<float> pointNKNSquaredDistance;
 
   std::cout << "K nearest neighbor search at (" << searchPoint.x 
             << " " << searchPoint.y 
             << " " << searchPoint.z
             << ") with K=" << K << std::endl;
  // 开始最近K邻搜索
   if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
   {
     for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
       std::cout << "    "  <<   (*cloud)[ pointIdxNKNSearch[i] ].x 
                 << " " << (*cloud)[ pointIdxNKNSearch[i] ].y 
                 << " " << (*cloud)[ pointIdxNKNSearch[i] ].z 
                 << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
   }
 
   //*************************K近邻搜索（搜索半径范围内的点）************************
  // 创建半径范围内点索引的容器
   std::vector<int> pointIdxRadiusSearch;
    // 创建搜索到的点的距离容器
   std::vector<float> pointRadiusSquaredDistance;
 
   float radius = 256.0f * rand () / (RAND_MAX + 1.0f);
 
   std::cout << "Neighbors within radius search at (" << searchPoint.x 
       << " " << searchPoint.y 
       << " " << searchPoint.z
       << ") with radius=" << radius << std::endl;
 
 // 开始半径搜索
   if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
   {
     for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
       std::cout << "    "  <<   (*cloud)[ pointIdxRadiusSearch[i] ].x 
                 << " " << (*cloud)[ pointIdxRadiusSearch[i] ].y 
                 << " " << (*cloud)[ pointIdxRadiusSearch[i] ].z 
                 << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
   }
 
}
