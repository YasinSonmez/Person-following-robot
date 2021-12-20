#include <global_planner/planner_core.h>
#include <navfn/MakeNavPlan.h>
#include <boost/shared_ptr.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#define PI 3.14159265

namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;
/*
struct Position
{
    double x;
    double y;
    double z;
    Position() : x(0), y(0), z(0) {}
    Position(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

struct sort_position_type
{
    inline bool operator()(const Position &p1, const Position &p2)
    {
        double len_1 = sqrt(p1.x * p1.x + p1.y * p1.y);
        double len2 = sqrt(p2.x * p2.x + p2.y * p2.y);
        return (len_1 < len2);
    }
};
struct Cell
{
    int f, g, x, y;
    Cell() : f(0), g(0), x(0), y(0) {}
    Cell(int _f, int _g, int _x, int _y) : f(_f), g(_g), x(_x), y(_y) {}

    bool operator<(const Cell &rhs) const
    {
        return f > rhs.f;
    }
};
Position a_star(Position init, Position goal, nav_msgs::OccupancyGrid costmapMsg, int window_size, double theta);
*/
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