#include "pcl_test_core.h"
#include <iostream>

template <class T>
class Filter{
	public:
		//VoxeiGrid 
		void VoxelGridMethod(T& input_cloud);
		//SACSegmentation

		void SACSMethod(T& cloud_filtered,int maxIterations,float distanceThreshold,float planarThreshold);
		//EuclideanCluster
    void PassThrough(T& cloud_filtered,int type);

		void EucClusterMethod(T inputcloud,std::vector<pcl::PointIndices>& cluster_indices);
};


template <class T>
void Filter<T>::VoxelGridMethod(T& input_cloud){
  pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(input_cloud);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*input_cloud);
}

template <class T>
void Filter<T>::PassThrough(T& cloud_filtered,int type){
  // auto startTime = std::chrono::steady_clock::now();
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_filtered);
  //当线由车后延到车头这个方向，左右是y，前后是x，上下是z，右y是负的，左y是正的
    //y:-1 represent right,+1 represent left "VLP16 Y X Z"Y -> zuo you  X-> qian
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-3,3);
  /*pass.setFilterFieldName("x");
  pass.setFilterLimits(-1.2,0.8);*/
  pass.filter(*cloud_filtered);
  if(type == 1){
  pass.setFilterFieldName("x");//x:-1 represent down +1 represent forward
  pass.setFilterLimits(0,10);
  }else if(type == 2){
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0,100);
  }else{
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0,20);
  }
  pass.filter(*cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(-0.5,1);
  pass.filter(*cloud_filtered);
  // auto endTime = std::chrono::steady_clock::now();
  //   auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  //   std::cout << "PassThrough took " << elapsedTime.count() << " milliseconds" << std::endl;
}

template <class T>
void Filter<T>::SACSMethod(T& cloud_filtered,int maxIterations,float thre){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=cloud_filtered;
  int sum=0;
  int finalsum=0;
  //分割地面 plannerThreshold = -0.2
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_f (new pcl::PointCloud<pcl::PointXYZ>);//提取除开平面点云的剩余点云集合
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());//提取平面的点云集
  //pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (thre);

  //循环删除所有平面，提取剩余点云。
  /*int i=0, nr_points = (int) cloud_filtered->points.size ();
  //当拟合到的点少大于30%的总点数则视为平面，需要剔除，这部分在实际比赛中可以尝试只执行一次
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {*/
    // 从剩余的点云中分割最大平面组成部分
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);//TODO RANSC
   /* if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
 // */
 //    //// 分离内层
 //    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);//setNegative表示，删除自己设置的Indices
 
    // Write the planar inliers to disk
    extract.filter (*cloud_plane);//设置完之后执行filter才能实现
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
 
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);//true代表保留剩余的点，方便下次循环使用。
    extract.filter (*cloud_f);
    
    cloud_filtered = cloud_f;
  }




template <class T>
void Filter<T>::EucClusterMethod(T inputcloud,std::vector<pcl::PointIndices>& cluster_indices){
   // auto startTime = std::chrono::steady_clock::now();
   pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
   //让z为0，速度快
   /*for(size_t i = 0;i<inputcloud->points.size();i++){
    inputcloud->points[i].z=0;
   }*/
  tree->setInputCloud (inputcloud);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec; //聚类对象
  ec.setClusterTolerance (0.04); //设置邻近搜索的搜索半径为3cm
  ec.setMinClusterSize (3);    //设置一个聚类需要的最少点云数目为100
  ec.setMaxClusterSize (200);  //最多点云数目为20000
  ec.setSearchMethod (tree);     //设置点云的搜索机制
  ec.setInputCloud (inputcloud); //设置原始点云
  ec.extract (cluster_indices);  //从点云中提取聚类
  // auto endTime = std::chrono::steady_clock::now();
  //   auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  //   std::cout << "EucCluster took " << elapsedTime.count() << " milliseconds" << std::endl;
}
