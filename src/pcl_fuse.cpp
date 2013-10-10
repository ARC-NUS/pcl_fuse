#include <functional>
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

ros::Publisher pub;

class PointCloudFusion {
  protected:
    // This is primarily to save on typing(!)
    typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t_;

    // The fused point cloud itself
    point_cloud_t_            fused_cloud_;

    // Listener for tf frames
    tf::TransformListener     tf_listener_;

    // The name of the base frame
    std::string               base_frame_id_;

    // publish the fused cloud
    void publish_() const {
      // temporary PointCloud2 intermediary
      pcl::PCLPointCloud2 tmp_pc;

      // Convert fused from PCL native type to ROS
      pcl::toPCLPointCloud2(fused_cloud_, tmp_pc);
      sensor_msgs::PointCloud2 published_pc;
      pcl_conversions::fromPCL(tmp_pc, published_pc);

      published_pc.header.frame_id = base_frame_id_;

      // Publish the data
      pub.publish(published_pc);
    }

  public:
    PointCloudFusion(const std::string& base_frame_id) {
      set_base_frame_id(base_frame_id);
    }
    ~PointCloudFusion() { }

    // get the base frame id
    const std::string base_frame_id() const { return base_frame_id_; }

    // update base frame id - this will reset the fused point cloud
    void set_base_frame_id(const std::string& base_frame_id) {
      // clear current fused point cloud on a base frame change
      fused_cloud_.clear();

      // record new frame
      base_frame_id_ = base_frame_id;
    }

    // callback when a new point cloud is available
    void add_cloud(const sensor_msgs::PointCloud2& ros_pc)
    {
      // temporary PointCloud2 intermediary
      pcl::PCLPointCloud2 tmp_pc;

      // transform the point cloud into base_frame_id
      sensor_msgs::PointCloud2 trans_ros_pc;
      if(!pcl_ros::transformPointCloud(base_frame_id_, ros_pc, trans_ros_pc, tf_listener_)) {
        // Failed to transform
        ROS_WARN("Dropping input point cloud");
        return;
      }

      // Convert ROS point cloud to PCL point cloud
      // See http://wiki.ros.org/hydro/Migration for the source of this magic.
      pcl_conversions::toPCL(trans_ros_pc, tmp_pc);

      // Convert point cloud to PCL native point cloud
      point_cloud_t_ input;
      pcl::fromPCLPointCloud2(tmp_pc, input);

      // Fuse
      fused_cloud_ += input;

      // Publish fused cloud
      publish_();
    }
};

// This is to save on typing
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;

void cloud_cb (const sensor_msgs::PointCloud2& ros_pc)
{
  // See http://wiki.ros.org/hydro/Migration for the source of this magic.
  pcl::PCLPointCloud2 pcl_pc;               // temporary PointCloud2 intermediary
  pcl_conversions::toPCL(ros_pc, pcl_pc);

  // Convert point cloud to PCL native point cloud
  point_cloud_t::Ptr input_ptr(new point_cloud_t());
  pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

  // Set up VoxelGrid filter to bin into 10cm grid
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(input_ptr);
  sor.setLeafSize(0.1, 0.1, 0.1);

  // Create output point cloud
  point_cloud_t::Ptr output_ptr(new point_cloud_t());

  // Run filter
  sor.filter(*output_ptr);

  // Now covert output back from PCL native type to ROS
  sensor_msgs::PointCloud2 ros_output;
  pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
  pcl_conversions::fromPCL(pcl_pc, ros_output);

  // Publish the data
  pub.publish(ros_output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  PointCloudFusion fusion("/odom");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, &PointCloudFusion::add_cloud, &fusion);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/fused_points", 1);

  // Spin
  ros::spin ();
}

