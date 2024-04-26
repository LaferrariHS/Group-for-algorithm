#ifndef DSTAR_PLANNER_CPP
#define DSTAR_PLANNER_CPP

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <dstar_global_planner/Dstar.h>
#include <dstar_global_planner/pathSplineSmoother/pathSplineSmoother.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/base_global_planner.h>


namespace dstar_global_planner{

  class DStarPlannerROS : public nav_core::BaseGlobalPlanner{

    public:

      DStarPlannerROS();

      DStarPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);



      ~DStarPlannerROS();


      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      void initialize(const std::string& name, costmap_2d::Costmap2D* costmap, std::string frame_id);

      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      bool getPlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


  protected:

      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
      vector<RealPoint> SmoothPlan(list<state> path);

      std::string global_frame_id_;

      costmap_2d::Costmap2D* costmap_;

      ros::Publisher plan_pub_;


      bool initialized_;
      bool smooth;

    private:

      Dstar *dstar_planner_; ///<  Dstar planner
      PathSplineSmoother *spline_smoother_;
      std::string tf_prefix_;
      int smooth_size_;

      void mapToWorld(double mx, double my, double& wx, double& wy);

      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);








  };
}
#endif


