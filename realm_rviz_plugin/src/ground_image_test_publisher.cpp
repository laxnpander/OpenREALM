#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <ros/package.h>
#include <geodesy/wgs84.h>
#include <pcl/point_types.h>
#include <geodesy/utm.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <realm_msgs/GroundImageCompressed.h>
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

ros::Publisher pub;

void stickCallback(const realm_msgs::GroundImageCompressedPtr img) {

    /*ROS_INFO("callb");

    custom_msgs::StitchedImageCompressed geo_image; // WARNING: custom_msgs does not exist anymore.

	std::cout << img->imagedata.format << std::endl;

	cv::Mat matrix = cv::imdecode(cv::Mat(img->imagedata.data),1);


    cv::namedWindow( "Display window", cv::WINDOW_NORMAL );// Create a window for display.

    cv::imshow( "Display window", matrix );                   // Show our image inside it.
    cv::waitKey(33);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", matrix).toImageMsg();

	geo_image.image = *msg.get();
	geographic_msgs::GeoPose p;



	p.position.latitude = img->gpsdata.latitude;
	p.position.altitude = img->gpsdata.altitude;
	p.position.longitude = img->gpsdata.longitude;
	p.orientation.x = img->orientation.x;
	p.orientation.y = img->orientation.y;
	p.orientation.z = img->orientation.z;
	p.orientation.w = img->orientation.w;

	geo_image.header.frame_id = "/image";

	geo_image.pose = p;
    geo_image.resolution = img->resolution;
	geo_image.header.stamp = ros::Time::now();

	geodesy::UTMPoint position_umt;
	geodesy::convert(geo_image.pose.position, position_umt);

	std::cout << "lat: " << img->gpsdata.latitude << "\t long: " << img->gpsdata.longitude << std::endl;
	std::cout << "umt east : " << position_umt.easting << "\t north: " << position_umt.northing << std::endl;

    pub.publish(geo_image);*/

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;

	ros::Publisher pub_filteredImageData = nh.advertise <pcl::PointCloud<pcl::PointXYZRGB> > ("/denseCloud",1,true);

    pub = nh.advertise<realm_msgs::GroundImageCompressed>("camera/ground_image", 1);
	ros::Subscriber sub = nh.subscribe("/stitched_image", 1000, stickCallback);

	cv::Mat image = cv::imread("/home/blume/Bilder/231.png", cv::IMREAD_UNCHANGED);

	std::cout << "Channels "<< image.channels() << std::endl;

//  cv::Mat image;
//
//  cv::cvtColor(image2,image,CV_8UC4);

	//load pcl
	std::ifstream ifile;
	ifile.open("/home/blume/dense.txt", std::ios::in);

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	double num = 0.0;

	for(int j = 0; j< 955380;j++)
	{
		try{
			pcl::PointXYZRGB point;
			ifile>> point.x;
			ifile>> point.y;
			ifile>> point.z;

			int color;

			ifile>> color;
			point.r = color;
			ifile>> color;
			point.g = color;
			ifile>> color;
			point.b = color;

			cloud.points.push_back(point);
		}catch(...)
		{
			break;
		}

	}

	std::cout << "load completed\n";

	for(int i = 0; i < cloud.size(); i++)
	{
		geodesy::UTMPoint position_umt;
		geographic_msgs::GeoPoint position_wb;

		position_wb.latitude = cloud.points.at(i).y;
		position_wb.longitude = cloud.points.at(i).x;
		position_wb.altitude =  cloud.points.at(i).z;
		geodesy::convert(position_wb, position_umt);

		if(i == 0)
		{
			std::cout <<"GPS: " <<  cloud.points.at(i).x << "  "<< cloud.points.at(i).y << std::endl;
			std::cout <<"cloud UTM east : "<< position_umt.easting << "  north: "<<position_umt.northing << std::endl;
		}

		cloud.points.at(i).x = position_umt.easting;
		cloud.points.at(i).y = position_umt.northing;
		cloud.points.at(i).z = 5;

	}
	cloud.header.frame_id = "/world";
	cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());

	std::cout << "Pub cloud "<< cloud.points.size() << "\n";

	pub_filteredImageData.publish(cloud);

//	cv::waitKey(30);
//	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgba8", image).toImageMsg();

//	ground_image_plugin::GeoImage geo_image;
//	geo_image.image = *msg.get();
//	geo_image.header.frame_id = "map";
//
//	sensor_msgs::NavSatFix fix;
//	fix.latitude = 10;
//	fix.longitude = 20;
//
//	geographic_msgs::GeoPose p;
//	p.position.latitude = 52.277666; //nord-süd
//	p.position.longitude = 11.491394; //ost-west
//	p.orientation.x = 0;
//	p.orientation.y = 0;
//	p.orientation.z = 0;
//	p.orientation.w = 1;
//
//	geo_image.pose = p;
//	geo_image.resolution = 1.0;  //pixel per meter //TODO berechnen
//
	tf::TransformBroadcaster br;
//
	tf::StampedTransform trans;
//
//	geographic_msgs::GeoPose p2;
//	p2.position.latitude = 52.2672; //nord-süd
//	p2.position.longitude = 10.5486; //ost-west
//	p2.orientation.x = 0;
//	p2.orientation.y = 0;
//	p2.orientation.z = 0;
//	p2.orientation.w = 1;
//	geodesy::UTMPoint position_umt;
//	geodesy::convert(p2.position, position_umt);
//	//std::cout << position_umt.easting << "  "<<position_umt.northing << std::endl;

	trans.frame_id_ = "/world";
	trans.child_frame_id_ = "/image";
	trans.setRotation(tf::Quaternion(0, 0, 0, 1));

	ROS_INFO_STREAM("Trans x: " << cloud.points.at(0).x << "y: " << cloud.points.at(0).y);

	trans.setOrigin(tf::Vector3(cloud.points.at(0).x, cloud.points.at(0).y, 0));



//	trans.stamp_ = t1;
//	geo_image.header.stamp = t1;


	//pub.publish(geo_image);
	cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
	pub_filteredImageData.publish(cloud);
	ros::Rate loop_rate(20);
	while (nh.ok()) {
		trans.stamp_ = ros::Time::now();
		br.sendTransform(trans);
//		cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
//		pub_filteredImageData.publish(cloud);

		ros::spinOnce();
		loop_rate.sleep();
	}
}

