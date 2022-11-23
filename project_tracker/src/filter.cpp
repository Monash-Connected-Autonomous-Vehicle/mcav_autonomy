#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/msg/point_cloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class Pcl2Filter : public rclcpp::Node
{
public:
  Pcl2Filter() : Node("pcl2_filter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 10, std::bind(&Pcl2Filter::filter_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_filtered", 10);
    groundPublisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl2Ground", 10);
  }

private:
  void publish_filtered_pcl2(const sensor_msgs::msg::PointCloud2 msg)
  {
    publisher_->publish(msg);
  }

  void passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr unfiltered,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr filtered,
                         std::string dimension,
                         float filterLowerBound,
                         float filterUpperBound,
                         bool setFilterLimitsNegative)
  {
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(unfiltered);
    filter.setFilterFieldName(dimension);
    filter.setFilterLimits(filterLowerBound, filterUpperBound);
    filter.setFilterLimitsNegative(setFilterLimitsNegative);
    filter.filter(*filtered);
  }

  sensor_msgs::msg::PointCloud2 pxyzPclToPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr pxyz)
  {
    pcl::PCLPointCloud2::Ptr PCLPc2(new pcl::PCLPointCloud2());
    sensor_msgs::msg::PointCloud2 pc2;

    pcl::toPCLPointCloud2(*pxyz, *PCLPc2);
    pcl_conversions::moveFromPCL(*PCLPc2, pc2);

    return pc2;
  }

  void filter_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "received msg");

    pcl::PCLPointCloud2::Ptr pclPc2_t(new pcl::PCLPointCloud2());

    pcl_conversions::toPCL(*msg, *pclPc2_t);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcxyz(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(*pclPc2_t, *pcxyz);

    // x and y-coordinate passthrough filter
    passThroughFilter(pcxyz, pcxyz, "x", -15.0, 15.0, false);
    passThroughFilter(pcxyz, pcxyz, "y", -15.0, 15.0, false);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcxyz_ground(new pcl::PointCloud<pcl::PointXYZ>);

    // z-coordinate passthrough filter, extracting the points on the ground
    passThroughFilter(pcxyz, pcxyz_ground, "z", -10.55, 1, true);

    auto pointCloud2Ground = pxyzPclToPointCloud2(pcxyz_ground);
    pointCloud2Ground.header.frame_id = "velodyne";
    groundPublisher_->publish(pointCloud2Ground);

    // z-coordinate passthrough filter
    passThroughFilter(pcxyz, pcxyz, "z", -1.55, 1, false);

    pcl::PCLPointCloud2::Ptr pclPc2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2 pclPc2_ds;

    // convert passthrough filtered pointcloud message to a PCLPointCloud2 message for downsampling
    pcl::toPCLPointCloud2(*pcxyz, *pclPc2);

    // downsampling the point cloud message
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(pclPc2);
    sor.setLeafSize(0.2f, 0.2f, 0.2f);
    sor.filter(pclPc2_ds);

    sensor_msgs::msg::PointCloud2 filtered_pc2;
    pcl_conversions::moveFromPCL(pclPc2_ds, filtered_pc2);

    // publising filtered pointcloud2 msg
    publish_filtered_pcl2(filtered_pc2);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groundPublisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pcl2Filter>());
  rclcpp::shutdown();
}
