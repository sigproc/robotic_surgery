#include "k2_client.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int pointBufferSize = 2605056;
int numberOfPoints = pointBufferSize / sizeof(float); // Hacky, much?
int streamSize = pointBufferSize + sizeof(double);

int main(int argC,char **argV)
{
	ros::init(argC,argV,"startPointCloud");
	ros::NodeHandle n;

	std::string serverAddress;
	n.getParam("/serverNameOrIP",serverAddress);
	Socket mySocket(serverAddress.c_str(),"9005",streamSize);

	ros::Publisher pub = n.advertise<PointCloud>("point_cloud",1);

	PointCloud::Ptr pc (new PointCloud);

	pc->header.frame_id =  ros::this_node::getNamespace().substr(1,std::string::npos) + "/kinect_pcl";
	while(ros::ok())
	{
		// TODO(Somhtr): change to ROS' logging API
		cout << "Reading data..." << endl;
		mySocket.readData();

		// TODO(Somhtr): change to ROS' logging API
		cout << "Copying data..." << endl;
		float* pt_coords = reinterpret_cast<float*>(mySocket.mBuffer);
		for(uint64_t idx=0; idx<numberOfPoints; idx+=3)
		{
			pc->push_back(pcl::PointXYZ(
				pt_coords[idx], pt_coords[idx+1], pt_coords[idx+2]
			));
		}

		double utcTime;
		memcpy(&utcTime,&mySocket.mBuffer[pointBufferSize],sizeof(double));
		pc->header.stamp = ros::Time(utcTime).toSec();

		pub.publish(pc);
		pc->clear();

		ros::spinOnce();
	}
	return 0;
}
