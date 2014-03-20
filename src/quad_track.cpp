#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#define MAX_CLUSTERS 1
#define PI 3.14159

class QuadTracker
{

private:
    ros::NodeHandle n_;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub[MAX_CLUSTERS];

public:
    QuadTracker()
    {
        ROS_DEBUG("Creating subscribers and publishers");
        // all publishers and subscribers:
        cloud_sub = n_.subscribe("box_filter/psy/output", 10,
                         &QuadTracker::cloudcb, this);

        int i = 0;
        for (i=0; i<MAX_CLUSTERS; i++)
        {
        std::stringstream ss;
        ss << "/cluster_" << i+1 << "_cloud";
        cloud_pub[i] = n_.advertise<sensor_msgs::PointCloud2>
            (ss.str(), 1);
        }

        return;
    }

    // this function gets called every time new pcl data comes in
    void cloudcb(const sensor_msgs::PointCloud2ConstPtr &input_cloud)
    {
        ROS_DEBUG("Filtered cloud receieved");
        ros::Time start_time = ros::Time::now();
        ros::Time tcur = ros::Time::now();
        ros::Time tcloud = (*input_cloud).header.stamp;

        sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2 ());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

        ROS_DEBUG("finished declaring vars : %f", (ros::Time::now()-tcur).toSec());
        tcur = ros::Time::now();

        // Convert to pcl
        ROS_DEBUG("Convert incoming cloud to pcl cloud");
        pcl::fromROSMsg(*input_cloud, *cloud);
        ROS_DEBUG("cloud transformed and converted to pcl : %f",
              (ros::Time::now()-tcur).toSec());
        tcur = ros::Time::now();

        ROS_DEBUG("Begin Planar extraction");
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.04);

        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);

       // Remove the planar inliers, extract the rest
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_filtered);

        ////////////////////////////////////////
        //    STARTING CLUSTER EXTRACTION     //
        ////////////////////////////////////////
        ROS_DEBUG("Begin cluster extraction");

        // create a vector for storing the indices of the clusters
        std::vector<pcl::PointIndices> cluster_indices;

        // setup cluster extraction:
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.01); 
        ec.setMinClusterSize (10);
        ec.setMaxClusterSize (500);
        ec.setInputCloud (cloud_filtered);
        // perform cluster extraction
        ec.extract (cluster_indices);

        //ROS_INFO("%d",cluster_indices.size());

        if (cluster_indices.size()>0)
        {

            std::vector<pcl::PointIndices>::const_iterator it=cluster_indices.begin();

            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                cloud_cluster_1->points.push_back (cloud_filtered->points[*pit]); 

            cloud_cluster_1->width = cloud_cluster_1->points.size ();
            cloud_cluster_1->height = 1;
            cloud_cluster_1->is_dense = true;       

            // convert to rosmsg and publish:
            ROS_DEBUG("Publishing extracted cloud");
            pcl::toROSMsg(*cloud_cluster_1, *ros_cloud);
            ros_cloud->header.frame_id = "/camera_depth_optical_frame";
            ros_cloud->header.stamp = tcloud;
            cloud_pub[0].publish(ros_cloud);
        
        } else {

            ROS_INFO("QUADROTOR NOT FOUND!!!");
        }

    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "link_tracker");

    // turn on debugging
    // log4cxx::LoggerPtr my_logger =
    // log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // ros::console::g_level_lookup[ros::console::levels::Debug]);

    ros::NodeHandle n;

    ROS_INFO("Starting quad tracker...\n");
    QuadTracker tracker;
  
    ros::spin();
  
    return 0;
}
