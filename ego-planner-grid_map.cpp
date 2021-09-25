

<include file="$(find plan_manage)/launch/rviz.launch"/>
<include file="$(find plan_manage)/launch/topo_algorithm.xml">

// rosmsgshow traj_utils/Bspline{
// int32 drone_id
// int32 order
// int64 traj_id
// time start_time
// float64[] knots
// geometry_msgs/Point[] pos_pts
//   float64 x
//   float64 y
//   float64 z
// float64[] yaw_pts
// float64 yaw_dt
// }

nav_msgs/Path{
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseStamped[] poses
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
}


in CBS/
rosbag play path1_2021-09-16-23-33-29.bag


轨迹可视化-在程序中以path类型数据对路径进行pub

//以下是程序//////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////

static  string mesh_resource;
static double color_r, color_g, color_b, color_a, cov_scale, scale;

nav_msgs::Path             pathROS;
int time_index = 0;
geometry_msgs::Pose     poseROS
visualization_msgs::Marker meshROS;


//nav_msgs::Path
publishPath(Vector3d pose，int id){

    poseROS.position.x = pose(0);
    poseROS.position.y = pose(1);
    poseROS.position.z = pose(2);

    pathROS.header.frame_id = _frame_id;
    pathROS.header.stamp = ?;

    pathROS.poses.push_back(poseROS);
    pathPub.publish(pathROS);

}

void timerCallback(const ros::TimerEvent& /*event*/)
{
    time_index++;
    publishMeshes(CBS输出,time_index);
    publishPath(CBS输出，time_index);
};

//visualization_msgs::Marker
publishMeshes(CBS输出,time_index)
{

  meshROS.header.frame_id = _frame_id;
  meshROS.header.stamp = ?; 
  meshROS.ns = "mesh";
  meshROS.id = 0;
  meshROS.type = visualization_msgs::Marker::MESH_RESOURCE;
  meshROS.action = visualization_msgs::Marker::ADD;
  meshROS.pose.position.x = pose(0);
  meshROS.pose.position.y = pose(1);
  meshROS.pose.position.z = pose(2);
//   q(0) = msg->pose.pose.orientation.w;
//   q(1) = msg->pose.pose.orientation.x;
//   q(2) = msg->pose.pose.orientation.y;
//   q(3) = msg->pose.pose.orientation.z;
  if (cross_config)
  {
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0)    += 45.0*PI/180.0;
    q          = R_to_quaternion(ypr_to_R(ypr)); 
  }  
  meshROS.pose.orientation.w = 0;
  meshROS.pose.orientation.x = 0;
  meshROS.pose.orientation.y = 0;
  meshROS.pose.orientation.z = 0;
  meshROS.scale.x = 2.0;
  meshROS.scale.y = 2.0;
  meshROS.scale.z = 2.0;
  meshROS.color.a = color_a;
  meshROS.color.r = color_r;
  meshROS.color.g = color_g;
  meshROS.color.b = color_b;
  meshROS.mesh_resource = mesh_resource;
  meshPub.publish(meshROS); 
}

main{
    ros::init(argc, argv, "visualization");
    ros::NodeHandle n("~");

    n.param("mesh_resource", mesh_resource, std::string("package://odom_visualization/meshes/hummingbird.mesh"));
    n.param("color/r", color_r, 1.0);
    n.param("color/g", color_g, 0.0);
    n.param("color/b", color_b, 0.0);
    n.param("color/a", color_a, 1.0);
    n.param("origin", origin, false);  
    n.param("robot_scale", scale, 2.0);    
    n.param("frame_id",   _frame_id, string("world") ); 


    ros::NodeHandle node_
    ros::timer vis_timer = node_.createTimer(ros::Duration(0.11), timerCallback);
    pathPub   = n.advertise<nav_msgs::Path>(            "path",                100, true);  
    meshPub   = n.advertise<visualization_msgs::Marker>("robot",               100, true); 
}


////////////////////以上是程序////////////////////////////////////
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////


{



    launch文件:{
        总的-src/planner/plan_manage/launch/simple_run.launch{
            1<include file="$(find ego_planner)/launch/rviz.launch"/>

            2<include file="$(find ego_planner)/launch/swarm.launch"/>{
                建图-<node pkg ="map_generator"  type ="random_forest" >
                飞行-<include file="$(find ego_planner)/launch/run_in_sim.launch">
            }
        }
        
        关键-可视化一架无人机(id = 1,2 ..)</run_in_sim.launch">{
 
            1 <include file="$(find ego_planner)/launch/advanced_param.xml">{

            }

            2 <node pkg="ego_planner" name="drone_$(arg drone_id)_traj_server" type="traj_server" output="screen">{
                    <remap from="position_cmd" to="drone_$(arg drone_id)_planning/pos_cmd"/>
                    <remap from="~planning/bspline" to="drone_$(arg drone_id)_planning/bspline"/>
                    <param name="traj_server/time_forward" value="1.0" type="double"/>

                    文件-src/planner/plan_manage/src/traj_server.cpp{
                        订阅1轨迹、以及一个闹钟回调10ms/每次{
                            nh.subscribe("planning/bspline", 10, bsplineCallback(traj_utils::BsplineConstPtr msg)){

                            };
                        发布1个{
                            advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

                        }
                        
                    }
            }
            3<include file="$(find ego_planner)/launch/simulator.xml">{

            }
        }
    
    }
    topic{
        <drone0>{
            规划轨迹-/drone_0_ego_planner_node/optimal_list{
                Type: visualization_msgs/Marker{.......}

                Publishers: 
                * /drone_0_ego_planner_node (http://ede-HBL-WX9:42557/)
                Subscribers: 
                * /rviz (http://ede-HBL-WX9:34793/)

                文件：<src/planner/traj_utils/src/planning_visualization.cpp>{

                }

            }
            执行轨迹-/drone_0_odom_visualization/path{
                Type: nav_msgs/Path{
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        geometry_msgs/PoseStamped[] poses
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/Pose pose
            geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
            geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        }
                Publishers: 
                * /drone_0_odom_visualization (http://ede-HBL-WX9:38739/)

                Subscribers: 
                * /rviz (http://ede-HBL-WX9:34793/)

                文件：<src/uav_simulator/Utils/odom_visualization/src/odom_visualization.cpp>{
                    odom_callback()->给pathmsg赋值、发布
                }
            }


            视域膨胀地图-/drone_0_ego_planner_node/grid_map/occupancy_inflate

            机器人-/drone_0_odom_visualization/robot{
                Type: visualization_msgs/Marker

                Publishers: 
                * /drone_0_odom_visualization (http://ede-HBL-WX9:38739/)

                Subscribers: 
                * /rviz (http://ede-HBL-WX9:34793/)
            }


        }
    }
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
    //文件路径：src/uav_simulator/Utils/odom_visualization/src/odom_visualization.cpp

geometry_msgs::PoseStamped poseROS;
    
      // Path
  static ros::Time prevt = msg->header.stamp;
  if ((msg->header.stamp - prevt).toSec() > 0.1)
  {
    prevt = msg->header.stamp;
    pathROS.header = poseROS.header;
    pathROS.poses.push_back(poseROS);
    pathPub.publish(pathROS);
  }
}


}




data_stream{
Pass_0("/map_generator/global_cloud"){//全局初始点云地图
    src/uav_simulator/map_generator/src/random_forest_sensing.cpp

    int main(int argc, char** argv) {
    _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);}
}

Pass_1("/pcl_render_node/cloud"){//局部点云图
    src/uav_simulator/local_sensing/src/pcl_render_node.cpp
}
Pass_2()



在哪调用？重点是输入的参数和话题名字
“setOccup”
    AStar::initGridMap
    AStar::AstarSearch




inline void GridMap::setOccupied(Eigen::Vector3d pos) {
    posToIndex(pos, id);
    md_.occupancy_buffer_inflate_[id***] = 1;
}


inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

struct MappingParameters {}初始化？

mp.map_origin_怎么赋值


nav_msgs::OccupancyGrid
/grid_map/occupancy
/grid_map/occupancy_inflate
/drone_5_ego_planner_node/grid_map/occupancy_inflate/
/drone_5_odom_visualization/rob




}

rostopic_list{ 
/clicked_point
/corrections
/ego_planner_node/a_star_list
/ego_planner_node/global_list
/ego_planner_node/goal_point
/ego_planner_node/goal_point_array
/ego_planner_node/init_list
/ego_planner_node/optimal_list
/ego_planner_node/optimal_list_array
/force_disturbance
/grid_map/occupancy
/grid_map/occupancy_inflate
/grid_map/unknown
/initialpose
/map_generator/global_cloud
/moment_disturbance
/motors
/move_base_simple/goal
/odom_visualization/cmd
/odom_visualization/covariance
/odom_visualization/covariance_velocity
/odom_visualization/height
/odom_visualization/path
/odom_visualization/pose
/odom_visualization/robot
/odom_visualization/robot_array
/odom_visualization/sensor
/odom_visualization/trajectory
/odom_visualization/velocity
/pcl_render_node/cloud
/pcl_render_node/depth
/pcl_render_node/local_map
/planning/bspline
/planning/data_display
/planning/pos_cmd
/quadrotor_simulator_so3/imu
/rosout
/rosout_agg
/so3_cmd
/so3_control/imu
/tf
/tf_static
/traj_start_trigger
/visual_slam/odom
/waypoint_generator/waypoints
/waypoint_generator/waypoints_vis
}

Frame_worker{
launch{
    arg{
        地图尺寸、里程计类型（视觉还是激光）
    }


    advanced_param.xml{// 主程序参数
        地图尺寸、里程计类型（视觉or激光）
            视觉->camera内参
                赋参:depth_topic <<  "/pcl_render_node/depth"   

            激光-> 赋参:cloud_topic << "/pcl_render_node/cloud"

        无人机 maximum velocity and acceleration
        无人机 规划域大小
        无人机 飞行模式(2D Nav Goal || global waypoints)
            global waypoints->设点的数量，以及每个点的位置

    }

    pkg="ego_planner" type="traj_server"{//trajectory server
    "/position_cmd"
    "/odom_world"
    "traj_server/time_forward" 
    }

    pkg="waypoint_generator" type="waypoint_generator" {
    "/move_base_simple/goal"    
    "/traj_start_trigger"
    }

    simulator.xml{
    "odometry_topic"<-which定位方式
    地图选择（mockamap || random forest）

    mockamap{
        resolution、map size
        噪声频率、分形精度、填充、分形衰减
    }

    quadrotor_simulator{
        初始位置
        定位方式:"/visual_slam/odom"
        扰动（时间、力）
    }

    so3_control{
    }

    }
}


}

Detail_occupied_map{
    sensor_msgs::PointCloud2 cloud_msg



    struct GridNode //作搜索用的数据结构，含有f、g以及index索引值。应由gridemap为其初始化？
struct MappingData {}

class GridMap {
    getInflateOccupancy(Eigen::Vector3d pos) {
        return int(MappingData.occupancy_buffer_inflate_[toAddress(id)]);}//vector<char> occupancy_buffer_inflate_
}



GridMap::Ptr->getInflateOccupancy



void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}
}