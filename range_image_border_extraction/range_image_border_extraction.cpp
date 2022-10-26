 /* \代码作者：Bastian Steder */
 
 #include <iostream>
 
 #include <pcl/range_image/range_image.h>  				// 深度图头文件
 #include <pcl/io/pcd_io.h>								// IO头文件
 #include <pcl/visualization/range_image_visualizer.h>	// 深度图可视化头文件
 #include <pcl/visualization/pcl_visualizer.h>			// 点云可视化头文件
 #include <pcl/features/range_image_border_extractor.h>	// 深度图边界提取头文件
 #include <pcl/console/parse.h> 						// 控制台语句分析头文件
 #include <pcl/common/file_io.h> 						// 获取无拓展名文件
 
 typedef pcl::PointXYZ PointType;						// 点云类型
 
 // --------------------
 // -----参数-----
 // --------------------
 float angular_resolution = 0.5f;	// 角度分辨率
  //coordinate_frame = CAMERA_FRAME告诉系统x面向右，y向下，z轴向前
 pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
 bool setUnseenToMaxRange = false;	//不设置所有不能观察到的点都为远距离的点

 
 // --------------
 // -----帮助提示-----
 // --------------
 void 
 printUsage (const char* progName)
 {
   std::cout << "\n\nUsage: "<<progName<<" [options] <scene.pcd>\n\n"
             << "Options:\n"
             << "-------------------------------------------\n"
             << "-r <float>   angular resolution in degrees (default "<<angular_resolution<<")\n"
             << "-c <int>     coordinate frame (default "<< (int)coordinate_frame<<")\n"
             << "-m           Treat all unseen points to max range\n"
             << "-h           this help\n"
             << "\n\n";
 }
 
 // --------------
 // -----主程序-----
 // --------------
 int 
 main (int argc, char** argv)
 {
   // --------------------------------------
   // -----解析命令行参数-------------------
   // --------------------------------------
   if (pcl::console::find_argument (argc, argv, "-h") >= 0)
   {
     printUsage (argv[0]);
     return 0;
   }
   if (pcl::console::find_argument (argc, argv, "-m") >= 0)
   {
     setUnseenToMaxRange = true;//设置所有不能观察到的点都为远距离的点
     std::cout << "Setting unseen values in range image to maximum range readings.\n";
   }
   int tmp_coordinate_frame;
   if (pcl::console::parse (argc, argv, "-c", tmp_coordinate_frame) >= 0)
   {
     coordinate_frame = pcl::RangeImage::CoordinateFrame (tmp_coordinate_frame);
     std::cout << "Using coordinate frame "<< (int)coordinate_frame<<".\n";
   }
   if (pcl::console::parse (argc, argv, "-r", angular_resolution) >= 0)
     std::cout << "Setting angular resolution to "<<angular_resolution<<"deg.\n";
   angular_resolution = pcl::deg2rad (angular_resolution);
   
   // ------------------------------------------------------------------
   // -----读取pcd文件或创建示例点云(如果没有给出)-----------------------
   // ------------------------------------------------------------------
   pcl::PointCloud<PointType>::Ptr point_cloud_ptr (new pcl::PointCloud<PointType>);
   pcl::PointCloud<PointType>& point_cloud = *point_cloud_ptr;
   pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
   Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
   std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
   if (!pcd_filename_indices.empty ())
   {
     std::string filename = argv[pcd_filename_indices[0]];
     if (pcl::io::loadPCDFile (filename, point_cloud) == -1)
    {
       std::cout << "Was not able to open file \""<<filename<<"\".\n";
       printUsage (argv[0]);
       return 0;
     }
     // 配置平移和旋转矩阵来设定传感器位姿
     scene_sensor_pose = Eigen::Affine3f (Eigen::Translation3f (point_cloud.sensor_origin_[0],
                                                                point_cloud.sensor_origin_[1],
                                                                point_cloud.sensor_origin_[2])) *
                         Eigen::Affine3f (point_cloud.sensor_orientation_);
  
     std::string far_ranges_filename = pcl::getFilenameWithoutExtension (filename)+"_far_ranges.pcd";
     if (pcl::io::loadPCDFile(far_ranges_filename.c_str(), far_ranges) == -1)
       std::cout << "Far ranges file \""<<far_ranges_filename<<"\" does not exists.\n";
   }
   else
   {
   // 如果pcd文件不存在则创建点云数据
     std::cout << "\nNo *.pcd file given => Generating example point cloud.\n\n";
     for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
       for (float y=-0.5f; y<=0.5f; y+=0.01f)
       {
        PointType point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
        point_cloud.push_back (point);
      }
    }
    point_cloud.width = point_cloud.size ();  point_cloud.height = 1;
  }
  
  // -----------------------------------------------
  // -----利用点云创建深度图像，方法见上一节-----
  // -----------------------------------------------
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;   
  range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                                   scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
  range_image.integrateFarRanges (far_ranges);
  if (setUnseenToMaxRange)
    range_image.setUnseenToMaxRange ();  //设置所有不能观察到的点都为远距离的点

  // --------------------------------------------
  // ---------使用3D点云查看器来查看源点云----------
  // --------------------------------------------
  pcl::visualization::PCLVisualizer viewer ("3D Viewer"); 	// 窗口名
  viewer.setBackgroundColor (1, 1, 1);						// 背景色 
 viewer.addCoordinateSystem (1.0f, "global");				// 添加全局坐标系
  pcl::visualization::PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 0, 0, 0); // 设置点云颜色
  viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  //PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 150, 150, 150);
  //viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
  //viewer.setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "range image");
  
  // -------------------------
  // -----使用深度图提取边界点-----
  // -------------------------
  pcl::RangeImageBorderExtractor border_extractor (&range_image); 	// 实例化深度图边界点提取器
  pcl::PointCloud<pcl::BorderDescription> border_descriptions;		// 实例化边界描述子
  border_extractor.compute (border_descriptions);					// 开始利用深度图计算边界，并返回给边界描述子。
  
  // ----------------------------------
  // -----使用3D查看器显示点云的物体边界点，阴影边界点，插值点-----
  // ----------------------------------
  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
                                      & veil_points = * veil_points_ptr,
                                      & shadow_points = *shadow_points_ptr;
  for (int y=0; y< (int)range_image.height; ++y)
  {
    for (int x=0; x< (int)range_image.width; ++x)
    {
    // 判断深度图中的点并提取出的这些点的类型，按类型分别装入不同点云容器中
      if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
        border_points.push_back (range_image[y*range_image.width + x]);
      if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
        veil_points.push_back (range_image[y*range_image.width + x]);
      if (border_descriptions[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
        shadow_points.push_back (range_image[y*range_image.width + x]);
    }
  }
  // 为不同类型的点着色，并渲染点的尺寸，并在显示器中显示出来
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
  viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
  viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
  viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");
  
  //-------------------------------------------
  // ------------显示深度图像------------------
  // ------------------------------------------
  pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
  range_image_borders_widget =
    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
                                                                          border_descriptions, "Range image with borders");
 // -------------------------------------
  
  
  //--------------------
  // ----开始循环显示-----
  //--------------------
  while (!viewer.wasStopped ())
 {
    range_image_borders_widget->spinOnce ();
    viewer.spinOnce ();
    pcl_sleep(0.01);
  }
}
