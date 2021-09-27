#include <target_generation_node.h>
using namespace std;
TargetNode::TargetNode()
{
	// Subscribers
	modelStates_sub = nh.subscribe("/gazebo/model_states", 1, &TargetNode::modelStatesCallback, this);
	odom_sub = nh.subscribe("/RosAria/odom", 1, &TargetNode::odomCallback, this);
	costmap_sub = nh.subscribe("/move_base_benchmark/global_costmap/costmap",
							   1, &TargetNode::costmapCallback, this);

	// Publishers
	target_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	followed_person_pub = nh.advertise<geometry_msgs::PoseStamped>("/followed_person", 10);
}

/****************************************FUNCTIONS***********************************************
************************************************************************************************/

// Get index from the string array by comparing a string
int getIndex(std::vector<std::string> v, std::string value)
{
	for (int i = 0; i < v.size(); i++)
	{
		if (v[i].compare(value) == 0)
			return i;
	}
	return -1;
}

// Takes a coordinate in map frame and transforms into idx corresponding in global costmap
int mapToCostmapIdx(double x, double y, int width, int height, double resolution)
{
	int a = (int)(y / resolution + (double)height / 2.0);
	int b = (int)(x / resolution + (double)width / 2.0);
	return a * width + b;
}

/****************************************CALLBACKS***********************************************
************************************************************************************************/

void TargetNode::odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
	odom = *odom_msg;
	odom_in_map_frame.header = odom.header;
	odom_in_map_frame.pose = odom.pose.pose;

	tf2_ros::TransformListener tf2_listener(tf_buffer);
	if (tf_buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(2.0)))
	{
		odom_to_map = tf_buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
		tf2::doTransform(odom_in_map_frame, odom_in_map_frame, odom_to_map);
	}
}

void TargetNode::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg)
{
	costmap = *costmap_msg;
}

void TargetNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states)
{
	int actor1_index = getIndex(model_states->name, "actor1");
	actor_pose.pose = model_states->pose[actor1_index];
	actor_pose.header.stamp = ros::Time::now();
	actor_pose.header.frame_id = "map";
	followed_person_pub.publish(actor_pose);
	actor_message_arrived = true;
}

/****************************************MAIN LOOP***********************************************
************************************************************************************************/
void TargetNode::mainLoop()
{
	// Check if the target position message is arrived
	if (actor_message_arrived)
	{
		// Get orientation from quaternion
		tf::Quaternion q(
			actor_pose.pose.orientation.x,
			actor_pose.pose.orientation.y,
			actor_pose.pose.orientation.z,
			actor_pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		// Substract PI/2.0 for convenience in the simulator
		yaw = yaw - PI / 2.0;
		target.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

		// Distance that is substracted from the position of the actor
		double distance_to_follow_behind = 1.5;

		if (FOLLOW_MODE == "between the robot and human")
		{
			// Stay some distance behind the human and between the robot and human
			double delta_x = actor_pose.pose.position.x - odom_in_map_frame.pose.position.x;
			double delta_y = actor_pose.pose.position.y - odom_in_map_frame.pose.position.y;
			double delta_r = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

			target.pose.position.x = actor_pose.pose.position.x -
									 distance_to_follow_behind * delta_x / delta_r;
			target.pose.position.y = actor_pose.pose.position.y -
									 distance_to_follow_behind * delta_y / delta_r;
		}
		else if (FOLLOW_MODE == "behind the human")
		{
			// Stay exactly behind the human by substracting in the direction where he looks
			target.pose.position.x = actor_pose.pose.position.x -
									 distance_to_follow_behind * cos(yaw);
			target.pose.position.y = actor_pose.pose.position.y -
									 distance_to_follow_behind * sin(yaw);
		}
		else if (FOLLOW_MODE == "check costmap behind the human")
		{
			// If the costmap behind the human is occupied check the arc that is 1m behind
			// the human
			double delta_yaw = 0;
			int costmap_idx = 0;

			for (int i = 0; i < 180; i++)
			{
				delta_yaw = PI / 180.0 * i;
				// Check plus delta_yaw
				target.pose.position.x = actor_pose.pose.position.x -
										 distance_to_follow_behind * cos(yaw + delta_yaw);
				target.pose.position.y = actor_pose.pose.position.y -
										 distance_to_follow_behind * sin(yaw + delta_yaw);

				costmap_idx = mapToCostmapIdx(target.pose.position.x,
											  target.pose.position.y,
											  costmap.info.width,
											  costmap.info.height,
											  costmap.info.resolution);
				if ((int)costmap.data[costmap_idx] == 0)
					break;

				// Check minus delta_yaw
				target.pose.position.x = actor_pose.pose.position.x -
										 distance_to_follow_behind * cos(yaw - delta_yaw);
				target.pose.position.y = actor_pose.pose.position.y -
										 distance_to_follow_behind * sin(yaw - delta_yaw);

				costmap_idx = mapToCostmapIdx(target.pose.position.x,
											  target.pose.position.y,
											  costmap.info.width,
											  costmap.info.height,
											  costmap.info.resolution);
				if ((int)costmap.data[costmap_idx] == 0)
					break;
			}
		}
		target.pose.position.z = 0;

		// Header info
		target.header.stamp = ros::Time::now();
		target.header.frame_id = "map";
		// Publish the target
		target_pub.publish(target);
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_generation");

	TargetNode node;
	ros::Rate loop_rate(2);

	while (ros::ok())
	{
		node.mainLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}