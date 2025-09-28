#include <pluginlib/class_list_macros.h>

#include <virtual_wall/virtual_wall.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(virtual_wall::VirtualWall, costmap_2d::Layer)

using namespace std;

namespace virtual_wall{

VirtualWall::VirtualWall()
{
    wallmax_x = 0.0;
    wallmax_y = 0.0;
    wallmin_x = 0.0;
    wallmin_y = 0.0;
}

VirtualWall::~VirtualWall()
{
}

void VirtualWall::onInitialize()
{
    boost::unique_lock < boost::recursive_mutex > lock(data_access_);
    ros::NodeHandle nh("~/" + name_), g_nh;;
    matchSize();
    current_ = true;
    enabled_ = true;
    nh.param("map_frame", map_frame_, std::string("map"));
    add_wall_sub_ = g_nh.subscribe("add_wall", 1, &VirtualWall::AddWallCallback, this);
    delete_wall_sub = g_nh.subscribe("delete_wall", 1, &VirtualWall::DeleteWallCallback, this);
}

void VirtualWall::matchSize()
{
    boost::unique_lock < boost::recursive_mutex > lock(data_access_);
    costmap_2d::Costmap2D* master = layered_costmap_->getCostmap();
    resolution = master->getResolution();
    global_frame_ = layered_costmap_->getGlobalFrameID();
}

/* *********************************************************************
 * updateBounds
 *
 * update obstacles and bounds
 */
void VirtualWall::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
    *min_x = std::min(wallmin_x, *min_x);
    *min_y = std::min(wallmin_y, *min_y);
    *max_x = std::max(wallmax_x, *max_x);
    *max_y = std::max(wallmax_y, *max_y);
}

/* *********************************************************************
 * updateCosts
 *
 * updates the master grid in the area defined in updateBounds
 */
void VirtualWall::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  boost::unique_lock < boost::recursive_mutex > lock(data_access_);
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_->lookupTransform(global_frame_, map_frame_, ros::Time(0));
  }catch (tf2::TransformException ex){
    ROS_ERROR("%s", ex.what());
    return;
  }
  // Copy map data given proper transformations
  tf2::Transform tf2_transform;
  tf2::convert(transform.transform, tf2_transform);

  for (size_t i = 0; i < wallPoint.size(); i++)
  {
    double wx, wy;
    for (size_t j = 0; j < wallPoint[i].polygon.points.size(); j++)
    {
        unsigned int pixle_x;
        unsigned int pixle_y;
        wx = wallPoint[i].polygon.points[j].x;
        wy = wallPoint[i].polygon.points[j].y;
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform*p;
        bool ret = master_grid.worldToMap(p.x(), p.y(), pixle_x, pixle_y);
        if (ret)
        {
          master_grid.setCost(pixle_x, pixle_y, costmap_2d::LETHAL_OBSTACLE);
        }
    }
  }
}

//虚拟墙插补运算
bool VirtualWall::WallInterpolation()
{
  double pixle_x[2];
  double pixle_y[2];
  double k,b;
  
  for (size_t i = 0; i < 2; i++)
  {
    if (fabs(wallPoint.back().polygon.points[0].x - wallPoint.back().polygon.points[1].x) > fabs(wallPoint.back().polygon.points[0].y - wallPoint.back().polygon.points[1].y))
    {
      pixle_x[i] = wallPoint.back().polygon.points[i].x;
      pixle_y[i] = wallPoint.back().polygon.points[i].y;
    }else{
      pixle_x[i] = wallPoint.back().polygon.points[i].y;
      pixle_y[i] = wallPoint.back().polygon.points[i].x;
    }
  }

  k = (pixle_y[0] - pixle_y[1]) / (pixle_x[0] - pixle_x[1]);
  b = pixle_y[0] - k * pixle_x[0];
  // cout << "k : " << k << "b " << b << endl;

  wallPoint.back().polygon.points.clear();
  for (double i = std::min(pixle_x[0], pixle_x[1]); i < std::max(pixle_x[0], pixle_x[1]); i += resolution)
  {
    // cout << "i " << i << " , " << std::max(pixle_x[0], pixle_x[1]) << endl;
    geometry_msgs::Point32 point;
    if (fabs(v_wall.back().polygon.points[0].x - v_wall.back().polygon.points[1].x) > fabs(v_wall.back().polygon.points[0].y - v_wall.back().polygon.points[1].y))
    {
      point.x = i;
      point.y = k * i + b;
    }else{
      point.x = k * i + b;
      point.y = i;
    }
    // cout << i << "(" << point.x << " , " << point.y << ") ";
    wallPoint.back().polygon.points.push_back(point);
  }
  return true;
}

//添加虚拟墙
void VirtualWall::AddWallCallback(const geometry_msgs::PolygonStampedConstPtr& msg)
{
  v_wall.clear();
  wallPoint.clear();
  virtual_wall::Wall wall1;
  wall1.id = 0;
   
  wall1.polygon.points.push_back(msg->polygon.points[0]);
  wall1.polygon.points.push_back(msg->polygon.points[1]); 

  v_wall.push_back(wall1);
  wallPoint.push_back(v_wall.back());
  /*
  for (size_t i = 0; i < wallPoint.size(); i++)
  {
      for (size_t j = 0; j < wallPoint[i].polygon.points.size(); j++)
      {
      cout << wallPoint[i].polygon.points[j].x << endl;
      cout << wallPoint[i].polygon.points[j].y << endl;
      }
  }
  for (size_t i = 0; i < v_wall.size(); i++)
  {
      for (size_t j = 0; j < v_wall[i].polygon.points.size(); j++)
      {
      cout << v_wall[i].polygon.points[j].x << endl;
      cout << v_wall[i].polygon.points[j].y << endl;
      }
  }
  */
  //对虚拟墙插值
  WallInterpolation();
  
  virtual_wall::Wall wall2;
  wall2.id = 1;
   
  wall2.polygon.points.push_back(msg->polygon.points[1]);
  wall2.polygon.points.push_back(msg->polygon.points[2]); 

  v_wall.push_back(wall2);
  wallPoint.push_back(v_wall.back());
  //对虚拟墙插值
  WallInterpolation();

  virtual_wall::Wall wall3;
  wall3.id = 2;
   
  wall3.polygon.points.push_back(msg->polygon.points[2]);
  wall3.polygon.points.push_back(msg->polygon.points[3]); 

  v_wall.push_back(wall3);
  wallPoint.push_back(v_wall.back());
  //对虚拟墙插值
  WallInterpolation();
  
  virtual_wall::Wall wall4;
  wall4.id = 3;
   
  wall4.polygon.points.push_back(msg->polygon.points[3]);
  wall4.polygon.points.push_back(msg->polygon.points[0]); 

  v_wall.push_back(wall4);
  wallPoint.push_back(v_wall.back());
  //对虚拟墙插值
  WallInterpolation();

}

//删除虚拟墙
void VirtualWall::DeleteWallCallback(const std_msgs::Int32ConstPtr& msg)
{
  v_wall.clear();
  wallPoint.clear();
  /*
  for (size_t i = 0; i < v_wall.size(); i++)
  {
    if(v_wall[i].id == msg->data){
      v_wall.erase(v_wall.begin() + i);
      wallPoint.erase(wallPoint.begin() + i);
    }
  }
  */
}

}
