#include "rclcpp/rclcpp.hpp"
#include <string.h>
#include <tf2_ros/transform_listener.h>
// #include <pcl_ros/transforms.h>
#include "laser_geometry/laser_geometry.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <sensor_msgs/PointCloud.h>
#include "sensor_msgs/msg/point_cloud2.hpp"
// #include <sensor_msgs/point_cloud_conversion.h> 
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
// #include <dynamic_reconfigure/server.h>
// #include <ira_laser_tools/laserscan_multi_mergerConfig.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"

using namespace std;
using namespace pcl;
// using namespace laserscan_multi_merger;
class LaserscanMerger
{
public:
    LaserscanMerger(rclcpp::Node::SharedPtr);
    void scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan, std::string topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud, builtin_interfaces::msg::Time time_stamp);
#ifdef DYNAMIC_RECONFIG
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);
#endif
private:
    rclcpp::Node::SharedPtr node_;
    laser_geometry::LaserProjection projector_;
	std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
    vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;
    vector<string> input_topics;

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;
};

#ifdef DYNAMIC_RECONFIG
void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}
#endif

void LaserscanMerger::laserscan_topic_parser()
{
#if 1
	vector<string> tmp_input_topics;
	string scan_topic_0 = node_->declare_parameter("scan_topic_0", "/scan0");
	string scan_topic_1 = node_->declare_parameter("scan_topic_1", "/scan1");
	tmp_input_topics.push_back(scan_topic_0);
	tmp_input_topics.push_back(scan_topic_1);

	input_topics = tmp_input_topics;
	if(input_topics.size() > 0)
	{
		scan_subscribers.resize(input_topics.size());
		clouds_modified.resize(input_topics.size());
		clouds.resize(input_topics.size());
		RCLCPP_INFO(node_->get_logger(), "Subscribing to topics\t%ld", scan_subscribers.size());
		for(int i=0; i<input_topics.size(); ++i)
		{
			scan_subscribers[i] = node_->create_subscription<sensor_msgs::msg::LaserScan>(
				input_topics[i], 
				rclcpp::QoS(30).best_effort(),
				[=](sensor_msgs::msg::LaserScan::SharedPtr msg) 
				{
					LaserscanMerger::scanCallback(msg, input_topics[i]); 
				}
			);
			clouds_modified[i] = false;
			cout << input_topics[i] << " ";
		}
	}
	else
		RCLCPP_INFO(node_->get_logger(), "Not subscribed to any topic.");
#else
	// LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    istringstream iss(laserscan_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));
	vector<string> tmp_input_topics;
	for(int i=0;i<tokens.size();++i)
	{
        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_input_topics.push_back(topics[j].name);
			}
		}
	}

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());


	// Do not re-subscribe if the topics are the same
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<scan_subscribers.size(); ++i)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
            scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
            ROS_INFO("Subscribing to topics\t%ld", scan_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
                scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan> (input_topics[i].c_str(), 1, boost::bind(&LaserscanMerger::scanCallback,this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
            ROS_INFO("Not subscribed to any topic.");
	}
#endif
}

LaserscanMerger::LaserscanMerger(rclcpp::Node::SharedPtr nh)
{
	node_ = nh;

#if 1
    this->destination_frame       = node_->declare_parameter("destination_frame","laser_frame");
	this->cloud_destination_topic = node_->declare_parameter("cloud_destination_topic","/merged_cloud");
    this->scan_destination_topic  = node_->declare_parameter("scan_destination_topic", "/scan");
    this->laserscan_topics        = node_->declare_parameter("laserscan_topics", "");
    this->angle_min               = node_->declare_parameter("angle_min", -M_PI);
    this->angle_max               = node_->declare_parameter("angle_max",  M_PI);
    this->angle_increment         = node_->declare_parameter("angle_increment", 0.0058);
    this->scan_time               = node_->declare_parameter("scan_time", 0.0333333);
    this->range_min               = node_->declare_parameter("range_min", 0.45);
    this->range_max               = node_->declare_parameter("range_max", 25.0);

#else
    nh.param<std::string>("destination_frame", destination_frame, "cart_frame");
    nh.param<std::string>("cloud_destination_topic", cloud_destination_topic, "/merged_cloud");
    nh.param<std::string>("scan_destination_topic", scan_destination_topic, "/scan_multi");
    nh.param<std::string>("laserscan_topics", laserscan_topics, "");
    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);
#endif
    this->laserscan_topic_parser();
    rclcpp::QoS qos(rclcpp::KeepLast(10));
	point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_destination_topic, qos);
	laser_scan_publisher_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(scan_destination_topic, qos);

}

void LaserscanMerger::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan, std::string topic)
{
#if 1
	sensor_msgs::msg::PointCloud2 oriCloud, tmpCloud;
	// Verify that TF knows how to transform from the received scan to the destination scan frame
	if(tfBuffer_ == nullptr) {
		tfBuffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
		tfListener_ = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer_);
    	auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
			node_->get_node_base_interface(),
			node_->get_node_timers_interface());
    	tfBuffer_->setCreateTimerInterface(timer_interface);
	}
	bool callback_timeout = false;
	tfBuffer_->waitForTransform(scan->header.frame_id, this->destination_frame, scan->header.stamp,	tf2::durationFromSec(1.0),
		[this, &callback_timeout](const tf2_ros::TransformStampedFuture & future)
		{
			try {
				// Expect this to throw an exception due to timeout
				future.get();
			} catch (...) {
				RCLCPP_ERROR(node_->get_logger(), "waitForTransform callback timeout");
				callback_timeout = true;
			}
		}
	);
	projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan,	oriCloud, *tfBuffer_, this->range_max+1, laser_geometry::channel_option::Distance);
	try
	{
		geometry_msgs::msg::TransformStamped transformStamped = tfBuffer_->lookupTransform(destination_frame, scan->header.frame_id, scan->header.stamp);
		tf2::doTransform(oriCloud, tmpCloud, transformStamped);
	}catch (tf2::TransformException ex){RCLCPP_ERROR(node_->get_logger(),"%s",ex.what());return;}

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
			pcl_conversions::toPCL(tmpCloud, clouds[i]);
			clouds_modified[i] = true;
		}
	}

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
#if 1
		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud, points);
		pointcloud_to_laserscan(points, &merged_cloud, scan->header.stamp);

		pcl_conversions::moveFromPCL(merged_cloud, tmpCloud);
		point_cloud_publisher_->publish(tmpCloud);
#else
		pcl_conversions::moveFromPCL(merged_cloud, tmpCloud);
		point_cloud_publisher_->publish(tmpCloud);

		Eigen::MatrixXf points;
		pcl_conversions::toPCL(tmpCloud, merged_cloud); // Temporary workaround to filter wrong scan
		getPointCloudAsEigen(merged_cloud, points);

		pointcloud_to_laserscan(points, &merged_cloud, scan->header.stamp);
#endif
	}
#else
	sensor_msgs::msg::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::msg::PointCloud2 tmpCloud3;

	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}	

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}
	
		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
#endif
}

void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud, builtin_interfaces::msg::Time time_stamp)
{
	auto output = make_unique<sensor_msgs::msg::LaserScan>();
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = time_stamp;
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0);

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			RCLCPP_DEBUG(node_->get_logger(), "rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		float range_sq = y*y+x*x;
		float range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			RCLCPP_DEBUG(node_->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		float angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			RCLCPP_DEBUG(node_->get_logger(), "rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}
		int index = (angle - output->angle_min) / output->angle_increment;


		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}

	laser_scan_publisher_->publish(move(output));
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto nh = rclcpp::Node::make_shared("laserscan_multi_merger");

    LaserscanMerger _laser_merger(nh);
#ifdef DYNAMIC_RECONFIG
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

#endif
	rclcpp::spin(nh);
	return 0;
}