#include <iostream>
#include <Eigen/Eigen>
using namespace std;
int main()
{
    cout << "Hello, world!" << endl;
    return 0;
	Eigen::Vector3d dd;
	dd.x()  = 1;
	dd.y() = 2;
	dd.z() = 3;
	cout<<dd(1)<<endl;
}



${workspaceFolder}/**
/usr/local/include/
/usr/include/
/usr/include/eigen3/
/opt/ros/melodic/include/
/usr/include/pcl-1.8/pcl/
/opt/ros/melodic/include/pcl_ros/
/opt/ros/melodic/include/pcl_conversions/