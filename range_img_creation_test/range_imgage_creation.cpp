#include <pcl/range_image/range_image.h> // 深度图头文件
int main () {
  pcl::PointCloud<pcl::PointXYZ> pointCloud; // 点云
  
  // 创造矩形点云数据，0.01分辨率
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.push_back(point);
    }
  }
  pointCloud.width = pointCloud.size();
  pointCloud.height = 1;
  
  // 利用以上创造的矩形体点云，创造一个1度角分辨率的深度图像
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //  1角度转弧度
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360角度转弧度
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180角度转弧度
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f); // 传感器位姿roll = pitch = yaw = 0（6DOF视角位姿）
  // 深度图所使用的坐标系
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00; 
  float minRange = 0.0f; 
  int borderSize = 1;   
  
  pcl::RangeImage rangeImage; // 深度图对象
  // 从以上的点云和设置的参数来创建深度图
  // ================================下面函数参数解释如下======================================
  // [in] 点云、角分辨率1°、
  // [in]maxAngleWidth = 360和maxAngleHeight = 180表示我们正在模拟的深度传感器具有完整的360度周围视图。（注：深度图像将仅裁剪为自动观察到某些内容的区域）。也可以通过减少值来节省一些计算。例如，对于180度视图朝前的激光扫描仪，可以观察到传感器后面没有点，maxAngleWidth = 180就足够了。
  // [in]sensorPose将虚拟深度传感器的6DOF位置定义为roll = pitch = yaw = 0的原点。
  // [in]coordinate_frame = CAMERA_FRAME告诉系统x面向右，y向下，z轴向前。另一种选择是LASER_FRAME，x面向前方，y位于左侧，z位于上方。
  // [in]对于noiseLevel = 0，沿Z轴剖分创建深度图像。然而，想平均落在同一个单元格中的点数，可以使用更高的值。比如0.05表示所有与最近点的最大距离为5cm的点用于计算深度。
  // [in]minRange 如果minRange大于0，所有靠近虚拟深度传感器的点将被忽略。
  // [in]borderSize，当裁剪图像时，如果borderSize大于0将在图像四周留下未观察到的点的边界。
  // [return]rangeImage 深度图
  //=========================================================================================
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  std::cout << rangeImage << "\n";
}
