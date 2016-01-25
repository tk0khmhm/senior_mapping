#include <ros/ros.h>
#include <boost/thread.hpp>
#include <math.h>

#include <nav_msgs/Odometry.h>


using namespace std;


/////////////////////////////////////////////////
//                  Callback                   //
/////////////////////////////////////////////////
boost::mutex odom_mutex;
nav_msgs::Odometry odom_out;
ros::Publisher pub_odom_;
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odom_mutex);
	odom_out = *msg;
	odom_out.child_frame_id = "base_link_";
	odom_out.pose.pose.position.y = -odom_out.pose.pose.position.y;
	odom_out.twist.twist.angular.z = -odom_out.twist.twist.angular.z;
	odom_out.pose.pose.orientation.z = -odom_out.pose.pose.orientation.z;

	//cout<<"odom_in"<<endl;
	pub_odom_.publish(odom_out);
}
//////////////////////////////////////////////////
















int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_converter");
	ros::NodeHandle n;

	ros::Subscriber sub_odom = n.subscribe("tinypower/odom", 10, odomCallback);

	ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("tinypower/odom_", 10);
	pub_odom_ = pub_odom;



	ros::Rate r(10);
	while(ros::ok()){


		ros::spinOnce();
		r.sleep();
	}

	return 0;
}













