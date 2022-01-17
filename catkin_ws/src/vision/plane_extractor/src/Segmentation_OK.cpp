//ROS includes
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/conversions.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

ros::Publisher pub; //Publisher segmentation
ros::Publisher pub1;//Publisher voxel filter
ros::Publisher pub2;//Publisher passthrough filter
ros::Publisher pub3;//Publisher ransac filter
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container (Point Clouds) for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::PCLPointCloud2 cloud_voxel;
  pcl::PCLPointCloud2 cloud_passthrough;
  pcl::PCLPointCloud2 cloud_ransacfil;
 
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  /////////////////////Perform VoxelGrib filter////////////////////////////////
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);//0.015  0.015  0.015  
  sor.filter (cloud_filtered);

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2( cloud_filtered, point_cloud);
  pcl::copyPointCloud(point_cloud, *point_cloudPtr);

  /////////////////////Perform Passthrough filter/////////////////////////////

  //Create Passthrough object for the filter method
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_passed (new pcl::PointCloud<pcl::PointXYZRGB>); 

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (point_cloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0,1.2); //0 , 1.2
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_passed);

  ////////////////////////////Perform RANSAC filter////////////////////////////////

  //Create RANSAC object for the filter method
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ransac (new pcl::PointCloud<pcl::PointXYZRGB>);
   
   // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);

  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.02); //0.02

  seg1.setInputCloud (cloud_passed);
  seg1.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (cloud_passed);
  extract.setIndices (inliers);
  extract.setNegative (true);//true
  extract.filter (*cloud_ransac);
  
  ///////////////////////////////Filtering the remaining Table Plane ////////////////////////////WORKING ON IT!!

  //Create statistical object for the filter method
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_stplane (new pcl::PointCloud<pcl::PointXYZRGB>); 

   //Create the filtering object
  //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filtro;
  //filtro.setInputCloud (cloud_ransac);
  //filtro.setMeanK(35);
  //filtro.setStddevMulThresh(1.0);
  //filtro.filter (*cloud_stplane);

  ////////////////////////// Creating the KdTree object for the search method of the extraction/////////////////////////////////

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud(cloud_ransac);//cloud_ransac

  // create the extraction object for the clusters
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;

  // specify euclidean cluster parameters
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(150); //50
	ec.setMaxClusterSize(2750);//750
	ec.setSearchMethod(tree);//cloud ransac
	ec.setInputCloud(cloud_ransac);

// exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
	ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(new pcl::PointCloud<pcl::PointXYZRGB>);

  int j= 0;
 
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	  {
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		    {
                pcl::PointXYZRGB point;
                point.x = point_cloudPtr->points[*pit].x;
                point.y = point_cloudPtr->points[*pit].y;
                point.z = point_cloudPtr->points[*pit].z;

                if (j == 0) //Red	#FF0000	(255,0,0)
			     {
				      point.r = 0;
				      point.g = 0;
				      point.b = 255;
			     }
			    else if (j == 1) //Lime	#00FF00	(0,255,0)
			     {
				      point.r = 0;
				      point.g = 255;
				      point.b = 0;
			     }
			    else if (j == 2) // Blue	#0000FF	(0,0,255)
			     {
				      point.r = 255;
				      point.g = 0;
				      point.b = 0;
			     }
			    else if (j == 3) // Yellow	#FFFF00	(255,255,0)
			     {
				      point.r = 255;
				      point.g = 255;
				      point.b = 0;
			     }
			    else if (j == 4) //Cyan	#00FFFF	(0,255,255)
			     {
				      point.r = 0;
				      point.g = 255;
				      point.b = 255;
			     }
			    else if (j == 5) // Magenta	#FF00FF	(255,0,255)
			     {
				      point.r = 255;
				      point.g = 0;
				      point.b = 255;
			     }
			    else if (j == 6) // Olive	#808000	(128,128,0)
		     	 {
				      point.r = 128;
				      point.g = 128;
				      point.b = 0;
			     }
			    else if (j == 7) // Teal	#008080	(0,128,128)
			     {
				      point.r = 0;
				      point.g = 128;
				      point.b = 128;
			     }
			    else if (j == 8) // Purple	#800080	(128,0,128)
		     	 {
				      point.r = 128;
				      point.g = 0;
				      point.b = 128;
			     }
			    else
		   	     {
				      if (j % 2 == 0)
				       {
					        point.r = 255 * j / (cluster_indices.size());
					        point.g = 128;
					        point.b = 50;
				       }
				      else
				       {
					        point.r = 0;
					        point.g = 255 * j / (cluster_indices.size());
					        point.b = 128;
				       }
                 }
                point_cloud_segmented->push_back(point);
			
            }
        j++;
    }
  std::cerr<< "segemnted:  " << (int)point_cloud_segmented->size() << "\n";
  std::cerr<< "origin:     " << (int)point_cloudPtr->size() << "\n";

  // Convert to ROS data type
  point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
  if(point_cloud_segmented->size()) pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
  else pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);

  pcl::toPCLPointCloud2(*point_cloudPtr, cloud_voxel);
  pcl::toPCLPointCloud2(*cloud_passed, cloud_passthrough);
  pcl::toPCLPointCloud2(*cloud_ransac, cloud_ransacfil);

  
  sensor_msgs::PointCloud2 segmentation;
  sensor_msgs::PointCloud2 voxelfilter;
  sensor_msgs::PointCloud2 passthroughfilter;
  sensor_msgs::PointCloud2 ransacfilter;
  pcl_conversions::fromPCL(cloud_filtered, segmentation);
  pcl_conversions::fromPCL(cloud_voxel, voxelfilter);
  pcl_conversions::fromPCL(cloud_passthrough, passthroughfilter);
  pcl_conversions::fromPCL(cloud_ransacfil, ransacfilter);

  // Publish the data
  pub.publish (segmentation);
  pub1.publish (voxelfilter);
  pub2.publish (passthroughfilter);
  pub3.publish (ransacfilter);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "vision");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/kinect/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("segmentation", 1);
  pub1= nh.advertise<sensor_msgs::PointCloud2> ("voxelfilter",1);
  pub2= nh.advertise<sensor_msgs::PointCloud2> ("passthroughfilter",1);
  pub3= nh.advertise<sensor_msgs::PointCloud2> ("ransacfilter",1);

  // Spin
  ros::spin ();
}
