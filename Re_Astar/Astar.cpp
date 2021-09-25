#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <Eigen/Eigen>
#include <algorithm>
#include <math.h>

#include "backward.hpp"

#define inf 1>>20
typedef GridNode* GridNodePtr;

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::vector;
using std::make_pair;

bool _has_map = false;
AstarPathFinder* _astar_path_finder = new AstarPathFinder();

ros::Subscriber _map_sub;

struct GridNode
{   
    int id; // 1--> open set, -1 --> closed set
    Vector3d coord;
    Vector3i index; 
    double gScore, fScore;;

    GridNodePtr camefrom;
    GridNode(const Vector3i& _index,const Vector3d& _coord)
    {
        id = 0;
        index = _index;
        coord = _coord;

        gScore = inf;
        fScore = inf;
        camefrom = NULL;
    };

};


class AstarPathFinder
{	
	private:
	protected:
        GridNodePtr*** GridNodeMap;
        uint8_t * data;
        Vector3i startIdx;			// 起始点网格坐标
		Vector3i goalIdx;			// 目标点网格坐标

        Vector3d start_pt;			// 起始点世界坐标
		Vector3d goal_pt;			// 目标点世界坐标

		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		double resolution, inv_resolution;
        int GLXYZ_SIZE, GLYZ_SIZE;

        double gl_xl,gl_yl,gl_zl;
        double gl_xu,gl_yu,gl_zu;

        std::multimap<double, GridNodePtr> openSet;

        Vector3d gridIndex2coord(const Vector3i& index);
        Vector3i coord2gridIndex(const Vector3d& cor);
        
        void AstarGetSucc(GridNodePtr curr, vector<double>& _edgeCostSets, vector<GridNodePtr>& _neighborPtrSets);
        double getHeu(GridNodePtr _startPtr, GridNodePtr _gaolPtr);

    public:
        AstarPathFinder(){};
        ~AstarPathFinder(){};

        void setObs(const double& coord_x,const double& coord_y,const double& coord_z);
        void initGridMap(double resolution,Vector3d gl_l,Vector3d gl_u,Vector3i gl_max_idx);

        void AstarGraphSearch(const Vector3d& _start,const Vector3d& _end);

		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const Eigen::Vector3i & index) const;

        vector<Vector3d> getpath();
};

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;
    // 0.5是取格子的中心
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}



void AstarPathFinder::initGridMap(double _resolution,Vector3d gl_l,Vector3d gl_u,Vector3i gl_max_idx)
{
    gl_xl = gl_l(0);
    gl_yl = gl_l(1);
    gl_zl = gl_l(2);
    gl_xu = gl_u(0);
    gl_yu = gl_u(1);
    gl_zu = gl_u(2);

    GLX_SIZE = gl_max_idx(0);
    GLY_SIZE = gl_max_idx(1);
    GLZ_SIZE = gl_max_idx(2);
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE;  
    GLXYZ_SIZE = GLYZ_SIZE * GLX_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;

    data = new uint8_t[GLXYZ_SIZE];
    memset(data,0,GLXYZ_SIZE * sizeof(uint8_t));

    GridNodeMap = new GridNodePtr** [GLX_SIZE];
    for (int i = 0; i < GLX_SIZE; i++)
    {
        GridNodeMap[i] = new GridNodePtr* [GLY_SIZE];
        for (int j = 0; j < GLY_SIZE; j++)
        {
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for (int k = 0; k < GLZ_SIZE; k++)
            {
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx,pos);
            }
        }
    }
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{
    if(_has_map ) return;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 map_vis;
    pcl::fromROSMsg(pointcloud_map, cloud);
    if ((int)cloud.size() == 0) return;

    pcl::PointXYZ pt;
    for (int i = 0; i < (int)cloud.size(); i++)
    {
        pt = cloud.points[i];
        _astar_path_finder->setObs(pt.x,pt.y,pt.z);

    }
    _has_map = true;
}

void AstarPathFinder::setObs(const double& coord_x,const double& coord_y,const double& coord_z)
{
    if (coord_x < gl_xl || coord_y < gl_yl || coord_z < gl_zl || 
        coord_x > gl_xu || coord_y > gl_yu || coord_z > gl_zu)
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);

    data[idx_x * GLYZ_SIZE + idx_y * GLYZ_SIZE + idx_z] = 1;

}

void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<double>& _edgeCostSets, vector<GridNodePtr>& _neighborPtrSets)
{
    _edgeCostSets.clear();
    _neighborPtrSets.clear();

    int idxx = currentPtr->index(0,0);
    
}

double AstarPathFinder::getHeu(GridNodePtr _startPtr, GridNodePtr _gaolPtr)
{

}


void AstarPathFinder::AstarGraphSearch(const Vector3d& _start,const Vector3d& _end)
{   
    Vector3i start_idx = coord2gridIndex(_start);
    Vector3i end_idx = coord2gridIndex(_end);
    
    //初始化类的成员
    startIdx = start_idx;
    goalIdx = end_idx;
    start_pt = gridIndex2coord(start_idx);
    goal_pt = gridIndex2coord(end_idx);

    //声明目标节点
    GridNodePtr goalPtr = new GridNode(goalIdx,goal_pt);

    //初始化起始节点
    GridNodePtr startPtr = new GridNode(startIdx,start_pt);
    startPtr -> cameFrom = nullptr;
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr, goalPtr);
    openSet.clear();//good habit
    openSet.insert(make_pair(startPtr->fScore,startPtr));

    //添加节点扩展阶段的变量
    GridNodePtr currentPtr = NULL;
    GridNodePtr neighborPtr = NULL;
    vector<double> edgeCostSets;
    vector<GridNodePtr> neighborPtrSets;

    if (!openSet.empty())
    {
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());
        currentPtr->id = -1;
        //是目标值就结束
        if (currentPtr->index == goalIdx) {return;};
        // 非目标，开始扩展
        AstarGetSucc(currentPtr,edgeCostSets,neighborPtrSets);
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            GridNodePtr neighborPtr = neighborPtrSets[i];
            double edgeCost = edgeCostSets[i];

            if (neighborPtr->id == 0)
            {
                neighborPtr->id = 1;
                neighborPtr->gScore = currentPtr -> gScore + edgeCost;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,goalPtr);
                openSet.insert(make_pair(neighborPtr -> fScore,neighborPtr));
                continue;
            }

            else if (neighborPtr->id == 1)
            {
                if (currentPtr->gScore + edgeCost < neighborPtr->gScore)
                {       
                    neighborPtr->gScore = currentPtr->gScore + edgeCost;
                    neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,goalPtr);
                    neighborPtr->cameFrom = currentPtr;
                }
                continue;
            }

            else
                continue;                    
        }
    }
}


int main(int argc, char** argv)
{
    // 初始化名为demo_node的ros节点
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");

    _astar_path_finder = new AstarPathFinder();
    _map_sub  = nh.subscribe( "map", 1, rcvPointCloudCallBack);
    delete _astar_path_finder;

    
}