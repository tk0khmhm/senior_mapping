#include <ros/ros.h>
#include <boost/thread.hpp>
#include <math.h>

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>


#define	SAVE_NUM	80	// 点群の保存数
#define CONICAL_X	0.56
#define CONICAL_Y	0.0
#define CONICAL_Z	0.76


using namespace std;


/////////////////////////////////////////////////
//                  Callback                   //
/////////////////////////////////////////////////
boost::mutex odom_mutex;
nav_msgs::Odometry odom_in;
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odom_mutex);
	odom_in = *msg;
	//cout<<"odom_in"<<endl;
}


boost::mutex pc_mutex;
sensor_msgs::PointCloud pc_in;
sensor_msgs::PointCloud pc_out;
int i, t;
double theta;
double sin_theta, cos_theta;
double z, w;
int pc_count = 0;
ros::Publisher pub_pc_;
void pcCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
	boost::mutex::scoped_lock(pc_mutex);
	pc_in = *msg;
	t = pc_count*1081;
	z = odom_in.pose.pose.orientation.z;
	w = odom_in.pose.pose.orientation.w;

	if(w>0){
		theta = 2*asin(z);
	}else if(z>0){
		theta = 2*acos(w);
	}else{
		theta = -2*acos(w);
	}
	//cout<<"theta:"<<theta<<endl;

	sin_theta = sin(theta);
	cos_theta = cos(theta);
	for(i=0; i<1081; ++i){
		//cout<<i+t<<endl;
		if(pc_in.points[i].x == 0 && pc_in.points[i].y==0 && pc_in.points[i].z==0){
			//pc_out.points[i+t] = geometry_msgs::Point32(0.0, 0.0, 0.0);
			pc_out.points[i+t].x = 0.0;
			pc_out.points[i+t].y = 0.0;
			pc_out.points[i+t].z = 0.0;
		}else{
			pc_out.points[i+t].x = pc_in.points[i].x*cos_theta - pc_in.points[i].y*sin_theta;
			pc_out.points[i+t].y = pc_in.points[i].x*sin_theta + pc_in.points[i].y*cos_theta;
			pc_out.points[i+t].z = pc_in.points[i].z;

			pc_out.points[i+t].x += odom_in.pose.pose.position.x + CONICAL_X*cos_theta;
			pc_out.points[i+t].y += odom_in.pose.pose.position.y + CONICAL_X*sin_theta;
			pc_out.points[i+t].z += CONICAL_Z;

			pc_out.channels[0].values[i+t] = pc_in.channels[0].values[i];
		}
	}




	++pc_count;
	if(pc_count==SAVE_NUM){
		pc_count=0;
	}

	pub_pc_.publish(pc_out);
	//cout<<"pc_in"<<endl;
}



//////////////////////////////////////////////////
















int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_creator");
	ros::NodeHandle n;

	ros::Subscriber sub_pc = n.subscribe("eth_top/point_cloud_3", 10, pcCallback);
	ros::Subscriber sub_odom = n.subscribe("tinypower/odom", 10, odomCallback);

	ros::Publisher pub_pc = n.advertise<sensor_msgs::PointCloud>("senior/pointcloud", 10);
	pub_pc_ = pub_pc;

	//sensor_msgs::PointCloud pc_out;
	pc_out.header.frame_id = "/tiny_odom";
	pc_out.channels.resize(1);
	pc_out.channels[0].name = "intensity";
	pc_out.points.resize(SAVE_NUM*1081);
	pc_out.channels[0].values.resize(SAVE_NUM*1081);

	ros::Rate r(40);
	while(ros::ok()){

		//pub_pc.publish(pc_in);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}













