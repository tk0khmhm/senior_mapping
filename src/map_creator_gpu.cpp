#include <ros/ros.h>
#include <boost/thread.hpp>
#include <math.h>
#include <omp.h>

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>


#define RESOLUTION		0.02	// [m/個]
#define RESOLUTION_		1.0/RESOLUTION
#define WIDTH			6.0		// [m], map's meter
#define WIDTH_2			WIDTH/2.0
#define GRID_N			WIDTH/RESOLUTION		//map's grid number
#define GRID_NN			GRID_N*GRID_N			//map's grid number
#define ER				0.3		//map' obstacle expand radius
#define ERR				ER/RESOLUTION
#define CR				180.0	//circle resolution
#define CR__2			2.0/CR
#define LRF_OFFSET_X	0.0//0.193	//-0.35


using namespace std;




enum cost{FREE=0, LETHAL=100};


class MapIndex{
private:
	MapIndex();
public:
	MapIndex(int _i, int _j):i(_i),j(_j){	}
	MapIndex(const MapIndex& id):i(id.i),j(id.j){	}
	
	int i,j;
};
bool operator==(const MapIndex& lhs, const MapIndex& rhs){
	return ((lhs.i==rhs.i) && (lhs.j==rhs.j));
}

bool operator<(const MapIndex& lhs, const MapIndex& rhs){
	return ((1000*lhs.i+lhs.j) < (1000*rhs.i+rhs.j));
}



class ExpandMap{
private:

	list<MapIndex> expanded_circle;
	
	ExpandMap(const ExpandMap&);
	
public:
	ExpandMap(float _radius, float resolution);
	
	void expandObstacle(const nav_msgs::OccupancyGrid& map_in);
	void getGridCells(nav_msgs::GridCells& cells);
	
	nav_msgs::OccupancyGrid local_map;
};

// midpoint circle algorithm
ExpandMap::ExpandMap(float _radius, float resolution){
	int radius=round(_radius/resolution);

	int f=1-radius;
	int ddF_x=1;
	int ddF_y=-2*radius;
	int x=0;
	int y=radius;
	
	expanded_circle.push_back(MapIndex(0,radius));
	expanded_circle.push_back(MapIndex(0,-radius));
	expanded_circle.push_back(MapIndex(radius,0));
	expanded_circle.push_back(MapIndex(-radius,0));
	
	// draw circle line on grid map
	while(x<y){
		if(f>=0){
			y--;
			ddF_y+=2;
			f+=ddF_y;
		}
	
		x++;
		ddF_x+=2;
		f+=ddF_x;
		
		expanded_circle.push_back(MapIndex(x , y));
		expanded_circle.push_back(MapIndex(-x, y));
		expanded_circle.push_back(MapIndex(x ,-y));
		expanded_circle.push_back(MapIndex(-x,-y));
		if(x!=y){
		expanded_circle.push_back(MapIndex( y, x));
		expanded_circle.push_back(MapIndex(-y, x));
		expanded_circle.push_back(MapIndex( y,-x));
		expanded_circle.push_back(MapIndex(-y,-x));
		}
	}
	//cout<<"hello"<<endl;
	
	// fill the grids in the circle
	// bresenham's line algorithm
	int n=expanded_circle.size();
	list<MapIndex>::iterator itr=expanded_circle.begin();
	//for(itr=expanded_circle.begin(); itr!=expanded_circle.end(); itr++){
	for(int i=0; i<n; i+=2){
		int x1=itr->i;
		int y1=itr->j;
		itr++;
		int x2=itr->i;
		int y2=itr->j;
		itr++;
		bool steep= abs(y2-y1)>abs(x2-x1);
	
		if(steep){
			swap(x1,y1);
			swap(x2,y2);
		}
		if(x1>x2){
			swap(x1,x2);
			swap(y1,y2);		
		}
	
		int deltax=x2-x1;
		int deltay=abs(y2-y1);	
		float error=0;
		float deltaerr;
		if(deltax==0){
			deltaerr=10000;
		}else{
			deltaerr=deltay/deltax;	
		}
		int ystep;
		int yt=y1;
		if(y1<y2){
			ystep=1;
		}else{
			ystep=-1;
		}
		for(int xt=x1; xt<=x2; xt++){
			//cout<<xt<<":"<<x1<<","<<x2<<endl;
			if(steep){
				expanded_circle.push_back(MapIndex(yt,xt));
			}else{
				expanded_circle.push_back(MapIndex(xt,yt));
			}
		
			error+=deltaerr;
		
			if(error>=0.5){
				yt+=ystep;
				error-=1;
			}	
		}	
	}
	
	// delete several overlap grids
	expanded_circle.sort();
	expanded_circle.unique();
	
	//list<MapIndex>::iterator itr;
	/*
	for(itr=expanded_circle.begin(); itr!=expanded_circle.end(); itr++){
		cout<<itr->i<<"\t"<<itr->j<<endl;
	}*/
}

void ExpandMap::expandObstacle(const nav_msgs::OccupancyGrid& map_in){
	local_map=map_in;
	
	vector<int8_t>::iterator itr;
	for(itr=local_map.data.begin(); itr!=local_map.data.end(); itr++){
		*itr=FREE;
	}
	
	
	for(int xi=0; xi<(int)map_in.info.height; xi++){
		for(int yi=0; yi<(int)map_in.info.width; yi++){
			// if the cell is LETHAL
			if(map_in.data[xi+map_in.info.width*yi]!=FREE){
				// expand the LETHAL cells with respect to the circle radius
				list<MapIndex>::iterator litr;
				for(litr=expanded_circle.begin(); litr!=expanded_circle.end(); litr++){
					int x=xi+litr->i, y=yi+litr->j;
					if(x>=0 && x<(int)local_map.info.height && 	y>=0 && y<(int)local_map.info.width
						&& map_in.data[xi+map_in.info.width*yi]>local_map.data[x+map_in.info.width*y]){
						local_map.data[x+map_in.info.width*y]=map_in.data[xi+map_in.info.width*yi];
					}
				}
			}
		}
	}
}

//change(2011/09/30)
void ExpandMap::getGridCells(nav_msgs::GridCells& cells){
	cells.cells.clear();
	
	cells.header.frame_id=local_map.header.frame_id;
	cells.header.stamp=local_map.header.stamp;
	cells.cell_width=local_map.info.resolution;
	cells.cell_height=local_map.info.resolution;
	
	float _map_angle = tf::getYaw(local_map.info.origin.orientation);
	float map_angle = _map_angle - M_PI;
	
	
	for(int xi=0; xi<(int)local_map.info.height; xi++){
		for(int yi=0; yi<(int)local_map.info.width; yi++){
			if(local_map.data[xi+local_map.info.width*yi]!=FREE){
				float x_conv=cos(map_angle)*xi-sin(map_angle)*yi;
				float y_conv=sin(map_angle)*xi+cos(map_angle)*yi;
				
				float x=local_map.info.origin.position.x-x_conv*local_map.info.resolution;
				float y=local_map.info.origin.position.y-y_conv*local_map.info.resolution;//*/

				geometry_msgs::Point pt;
				pt.x=x;	pt.y=y;	pt.z=0;
				cells.cells.push_back(pt);
			}
		}
	}
}



/////////////////////////////////////////////////
//                  Callback                   //
/////////////////////////////////////////////////
boost::mutex odom_mutex;
//nav_msgs::Odometry odom_in_;
geometry_msgs::Vector3 odom_in;	// z:yaw
void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
	boost::mutex::scoped_lock(odom_mutex);
	//odom_in_ = *msg;
	odom_in.x = msg->pose.pose.position.x;
	odom_in.y = msg->pose.pose.position.y;
	odom_in.z = tf::getYaw(msg->pose.pose.orientation);
	//cout<<"odom_in"<<endl;
}


nav_msgs::OccupancyGrid cost_map;
boost::mutex pc_mutex;
size_t i, ii;
double x, y, j;
int p, q, q_, n;
void pcCallback(const sensor_msgs::PointCloudConstPtr& msg)
{
	boost::mutex::scoped_lock(pc_mutex);

	for(i=0; i<GRID_NN; i++){
		cost_map.data[i] = 0;
	}

	cost_map.info.origin.position.x = odom_in.x + WIDTH_2;
	cost_map.info.origin.position.y = odom_in.y + WIDTH_2;
	//cost_map.info.origin.orientation = odom_in_.pose.pose.orientation;

	for(i=0; i<msg->points.size(); i++){
		if(0.3<msg->points[i].z && msg->points[i].z < 0.5){
			x = -(msg->points[i].x - odom_in.x);
			y = -(msg->points[i].y - odom_in.y);

			p = (x+WIDTH_2)*RESOLUTION_;
			q = (y+WIDTH_2)*RESOLUTION_;
			q_ = q*GRID_N;

			if(fabs(x)<WIDTH_2 && fabs(y)<WIDTH_2){
				cost_map.data[ p + q_ ] = 100;
				//#ifdef _OPENMP
				//#pragma omp parallel for
				//#endif
				//for(j=0; j<CR; j++){
				//	n = p+(int)(ERR*cos(M_PI*j*CR__2)) + (q+(int)(ERR*sin(M_PI*j*CR__2)))*GRID_N;

				//	if(n>0 && n<GRID_NN){
				//		cost_map.data[n] = 100;
				//	}
				//}
			}

		}
	}

	//for(ii=0; ii<GRID_N; ii++){
	//	for(i=0; i<GRID_N; i++){
	//		if(cost_map.data[i+ii*GRID_N]==100){
	//			//cout<<"100"<<endl;
	//			q_ = ii*GRID_N;
	//			for(j=0; j<CR; j++){
	//				n = i+(int)(ERR*cos(M_PI*j*CR__2)) + (ii+(int)(ERR*sin(M_PI*j*CR__2)))*GRID_N;
	//				//cout<<"i:"<<i<<", ERR:"<<ERR<<", j:"<<j<<", CR__2:"<<CR__2<<", q_:"<<q_<<endl;
	//				//cout<<n<<endl;
	//				if(n>0 && n<GRID_NN){
	//					cost_map.data[n] = 100;
	//					//cout<<"inflate"<<endl;
	//				}
	//			}
	//		}
	//	}
	//}


	
	
}

//////////////////////////////////////////////////





int main(int argc, char** argv)
{
	ros::init(argc, argv, "map_creator");
	ros::NodeHandle n;

	ros::Subscriber sub_pc = n.subscribe("senior/pointcloud", 10, pcCallback);
	ros::Subscriber sub_odom = n.subscribe("tinypower/odom", 10, odomCallback);

	ros::Publisher pub_map = n.advertise<nav_msgs::OccupancyGrid>("senior/map", 10);

	cost_map.header.frame_id = "tiny_odom";
	cost_map.info.resolution = RESOLUTION;
	cost_map.info.width = GRID_N;
	cost_map.info.height = GRID_N;
	cost_map.info.origin.position.x = -WIDTH/2.0;
	cost_map.info.origin.position.y = -WIDTH/2.0;
	cost_map.info.origin.position.z = 0.0;
	cost_map.info.origin.orientation.z = 1;
	cost_map.data.resize(GRID_N*GRID_N);

	ExpandMap ex_map(3.0, 0.2);

	ros::Rate r(10);
	while(ros::ok()){

		ex_map.expandObstacle(cost_map);

		//pub_map.publish(cost_map);
		pub_map.publish(ex_map.local_map);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}













