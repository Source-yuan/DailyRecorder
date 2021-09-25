地图产生{
src/grid_path_searcher/src/random_complex_generator.cpp

_all_map_pub   = n.advertise<sensor_msgs::PointCloud2>("global_map", 1);

void RandomMapGenerate() 填装  globalMap_pcd
循环 void pubSensedPoints() 发布 "global_map" =
"grid_path_searcher/~map"
}


主node{
订阅两个topic{
_map_sub  = nh.subscribe( "map" = "/random_complex/global_map",1, rcvPointCloudCallBack ){
    1给pathfinder设置障碍物{
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {pt = cloud.points[idx];        
    _astar_path_finder->setObs(pt.x, pt.y, pt.z);}
    pt
    };
    2发布可视化信息{
        cloud_vis<-pt<-rcv
        pcl::toROSMsg(cloud_vis, map_vis);
        _grid_map_vis_pub.publish(map_vis);
    }
}
_pts_sub  = nh.subscribe( "waypoints" = "/waypoint_generator/waypoints", 1, rcvWaypointsCallback ){
    1获取rviz中鼠标点击目标值target_pt<<&wp;
    2做搜索pathFinding(_start_pt, target_pt){
    visGridPath (grid_path, false){
        _grid_path_vis_pub.publish(node_vis);
    };
    visVisitedNode(visited_nodes, false){
       _visited_nodes_vis_pub.publish(node_vis);
    };
    };
    3pathFinding()中有路径和受访点的可视化数据发布；
}
}

发布三个topic{
_grid_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1);//在订阅回调中发布了；
_grid_path_vis_pub = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1); 
_visited_nodes_vis_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis",1);
}

参数化变量-回调函数里用{
    nh.param("map/cloud_margin",  _cloud_margin, 0.0);
    // 分辨率，单个网格的尺寸。比如地图尺寸是50, 分辨率0.2情况下，共有250个网格
    nh.param("map/resolution",    _resolution,   0.2); 
    
    // 网格地图尺寸（物理尺度）
    nh.param("map/x_size",        _x_size, 50.0);
    nh.param("map/y_size",        _y_size, 50.0);
    nh.param("map/z_size",        _z_size, 5.0 );
    
    // 路径搜索的起始点，默认在(0,0,0)位置
    nh.param("planning/start_x",  _start_pt(0),  0.0);
    nh.param("planning/start_y",  _start_pt(1),  0.0);
    nh.param("planning/start_z",  _start_pt(2),  0.0);
}


_map_sub  = nh.subscribe( "map",1, rcvPointCloudCallBack );
rcvPointCloudCallBack{
pcl::toROSMsg(cloud_vis, map_vis);    
publish(map_vis);
}
}

rviz{

    订阅topic:/demo_node/grid_map_vis    
}


struct GridNode
{
    GridNode(){};
};


typedef GridNode* GridNodePtr;



class AstarPathFinder
{	
	private:

        uint8_t * data;

	protected:

        GridNodePtr*** GridNodeMap;

		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		double resolution, inv_resolution;
        int GLXYZ_SIZE, GLYZ_SIZE;

        double gl_xl,gl_yl,gl_zl;
        double gl_xu,gl_yu,gl_zu;

        Vector3d gridIndex2coord(const Vector3i& index)
        Vector3i coord2gridIndex(const Vector3d& cor)

    public:
        AstarPathFinder(){};
        ~AstarPathFinder(){};

        void setObs(const double& coord_x,const double& coord_y,const double& coord_z);
        void initGridMap(double resolution,Vector3d gl_l,Vector3d gl_u,Vector3i gl_max_idx);

        GridNodePtr search(cosnt Vector3d& start,const Vector3d& end);

		bool isOccupied(const Eigen::Vector3i & index) const;
		bool isFree(const Eigen::Vector3i & index) const;


        void expendedNode(GridNodePtr curr);
        void getHeu(cosnt Vector3d& start,const Vector3d& end);
        vector<Vector3d> getpath();
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
    GLYZ_SIZE = GLY_SIZE * GLZ_SIZE  
    GLXYZ_SIZE = GLYZ_SIZE * GLX_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution

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
