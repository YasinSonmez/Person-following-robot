#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;

namespace global_planner
{
    class PlannerWithCostmap : public GlobalPlanner
    {
    public:
        PlannerWithCostmap(string name, Costmap2DROS *cmap);
        bool makePlanService(navfn::MakeNavPlan::Request &req, navfn::MakeNavPlan::Response &resp);

    private:
        void poseCallback(const rm::PoseStamped::ConstPtr &goal);
        Costmap2DROS *cmap_;
        ros::ServiceServer make_plan_service_;
        ros::Subscriber pose_sub_;
    };
} // namespace