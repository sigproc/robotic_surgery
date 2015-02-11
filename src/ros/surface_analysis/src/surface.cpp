#include "surface.h"
using namespace std;

int main(int argc, char *argv[])
{
	// Initialise the surface analysis node

	ros::init(argc,argv,"surface");
	ros::NodeHandle node;

	ros::Subscriber sub = n.subscribe("depthImage",10, LoadDepth)
	return 0;
}

void LoadDepth(const sensor_msgs::Image::constptr msg)
{
	cout << "Depth image recieved" << endl;
}