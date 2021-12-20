/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/
#include "plan_utils.h"
namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using cm::Costmap2D;
using cm::Costmap2DROS;
using rm::PoseStamped;
using std::string;
using std::vector;
/*
using namespace std;

Position a_star(Position init, Position goal, nav_msgs::OccupancyGrid costmapMsg, int window_size = 0, double theta = 0)
{
    int delta[4][2] = {-1, 0, 0, -1, 1, 0, 0, 1};
    double resolution = costmapMsg.info.resolution;
    int height = costmapMsg.info.height;
    int width = costmapMsg.info.width;

    double originX = costmapMsg.info.origin.position.x;
    double originY = costmapMsg.info.origin.position.y;

    double topLimit = theta + PI / 2.0;
    double bottomLimit = theta - PI / 2.0;
    bool inverse = true;
    if (theta < -PI / 2.0)
    {
        topLimit = theta + 3 * PI / 2.0;
        bottomLimit = theta + PI / 2.0;
        inverse = false;
    }
    else if (theta > PI / 2.0)
    {
        topLimit = theta - PI / 2.0;
        bottomLimit = theta - 3 * PI / 2.0;
        inverse = false;
    }

    vector<vector<int>> costmap(height, vector<int>(width, 0));
    vector<vector<int>> heuristic(height, vector<int>(width, 0));

    for (int i = 0; i < height; i++)
        for (int j = 0; j < width; j++)
        {
            int currentData = costmapMsg.data[i * width + (width - 1 - j)];
            costmap[i][j] = currentData;
            heuristic[i][j] = abs(i - (int)goal.x) + abs(j - (int)goal.y);
        }

    vector<vector<int>> closed(height, vector<int>(width, 0));
    vector<vector<int>> action(height, vector<int>(width, 0));

    int x = init.x;
    int y = init.y;
    int g = costmap[x][y];
    int f = g + heuristic[x][y];
    priority_queue<Cell> cells;

    cells.push(Cell(f, g, x, y));
    bool found = false;
    bool resign = false;
    while (!found && !resign)
    {
        if (cells.size() == 0)
            return Position(-1, -1, -1);
        Cell next = cells.top();
        cells.pop();
        int x = next.x;
        int y = next.y;
        int g = next.g;

        if (x == (int)goal.x and y == (int)goal.y)
        {
            found = true;
        }
        else if (abs(x - (int)init.x) + abs(y - (int)init.y) > window_size && window_size)
        {
            found = true;

            cout << "Costmap index: " << x << " " << y << endl;
            double x_transformed = (-y - 0.5 + width) * resolution + originX;
            double y_transformed = (x + 0.5) * resolution + originY;
            return Position(x_transformed, y_transformed, 0);
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                int x2 = x + delta[i][0];
                int y2 = y + delta[i][1];
                if (x2 >= 0 && x2 < height and y2 >= 0 and y2 < width)
                {
                    if (closed[x2][y2] == 0)
                    {
                        double alpha = atan2(x2 - height / 2, -y2 + width / 2 + 1e-9);
                        bool isBack = (alpha > bottomLimit && alpha < topLimit) ^ inverse;
                        if (!isBack)
                        {
                            int g2 = costmap[x2][y2];
                            int f2 = g2 + heuristic[x2][y2];
                            cells.push(Cell(f2, g2, x2, y2));
                            closed[x2][y2] = 1;
                            action[x2][y2] = i;
                        }
                    }
                }
            }
        }
    }
    return Position(-1, -1, -1);
}*/
namespace global_planner
{
    bool PlannerWithCostmap::makePlanService(navfn::MakeNavPlan::Request &req, navfn::MakeNavPlan::Response &resp)
    {
        vector<PoseStamped> path;

        req.start.header.frame_id = "map";
        req.goal.header.frame_id = "map";
        bool success = makePlan(req.start, req.goal, path);
        resp.plan_found = success;
        if (success)
        {
            resp.path = path;
        }

        return true;
    }

    void PlannerWithCostmap::poseCallback(const rm::PoseStamped::ConstPtr &goal)
    {
        geometry_msgs::PoseStamped global_pose;
        cmap_->getRobotPose(global_pose);
        vector<PoseStamped> path;
        makePlan(global_pose, *goal, path);
    }

    PlannerWithCostmap::PlannerWithCostmap(string name, Costmap2DROS *cmap) : GlobalPlanner(name, cmap->getCostmap(), cmap->getGlobalFrameID())
    {
        ros::NodeHandle private_nh("~");
        cmap_ = cmap;
        make_plan_service_ = private_nh.advertiseService("make_plan", &PlannerWithCostmap::makePlanService, this);
        pose_sub_ = private_nh.subscribe<rm::PoseStamped>("goal", 1, &PlannerWithCostmap::poseCallback, this);
    }

} // namespace