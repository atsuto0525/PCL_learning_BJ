 #include <pcl/point_cloud.h>
 #include <pcl/octree/octree_pointcloud_changedetector.h>
 
 #include <iostream>
 #include <vector>
 #include <ctime>
 
 int
 main ()
{
  srand ((unsigned int) time (NULL));

  // 设置八叉树分辨率 即体素边长
  float resolution = 32.0f;

  // 实例化基于八叉树的点云变化检测类
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (resolution);

	// 创建点云A的指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZ> );

  // 为点云A生成一些随机点云数据
  cloudA->width = 128;
  cloudA->height = 1;
  cloudA->points.resize (cloudA->width * cloudA->height);
	
  for (std::size_t i = 0; i < cloudA->size (); ++i)
  {
    (*cloudA)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudA)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudA)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  }

  // 将点云A变成八叉树状结构
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();
  
// 点云A是我们的参考点云，用八叉树结构描述它的空间分布。
// OctreePointCloudChangeDetector类继承自Octree2BufBase类，
// 后者允许同时在内存中保存和管理两棵八叉树。
// 此外，它实现了一个内存池，可以重用已经分配的节点对象，因此在生成多个点云的八叉树时减少了昂贵的内存分配和回收操作。
//通过调用octree. switchbuffers()，我们重置了八叉树类，同时在内存中保留了之前的八叉树结构。

  // 八叉树缓冲区:重置八叉树，但在内存中保留以前的树结构。
  octree.switchBuffers ();
 // 创建点云B的指针
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZ> );
   
  // 为点云B生成一些随机点云数据
  cloudB->width = 128;
  cloudB->height = 1;
  cloudB->points.resize (cloudB->width * cloudB->height);

  for (std::size_t i = 0; i < cloudB->size (); ++i)
  {
    (*cloudB)[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudB)[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
    (*cloudB)[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
  }

  // 将点云B设置成八叉树结构
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();

// 为了检索存储在当前八叉树结构体素(基于clouddB)中的点，这些点在之前的八叉树结构(基于clouddA)中不存在，我们可以调用getPointIndicesFromNewVoxels方法，它返回结果点索引的向量。

	// 创建输出点云的索引容器
  std::vector<int> newPointIdxVector;

  // 从八叉树体素中获取点索引向量，这在之前的缓冲区中不存在
    octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points
  std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
  for (std::size_t i = 0; i < newPointIdxVector.size (); ++i)
    std::cout << i << "# Index:" << newPointIdxVector[i]
              << "  Point:" << (*cloudB)[newPointIdxVector[i]].x << " "
              << (*cloudB)[newPointIdxVector[i]].y << " "
              << (*cloudB)[newPointIdxVector[i]].z << std::endl;

}
