#include <target_generation_node.h>
using namespace std;
TargetNode::TargetNode()
{
	// Subscribers
	if (HUMAN_TRACKING_MODE == "laser")
		peopleTracked_sub = nh.subscribe("/people_tracked", 1, &TargetNode::peopleTrackedCallback, this);
	else if (HUMAN_TRACKING_MODE == "camera")
	{
		human_sub = nh.subscribe("human_pose", 1, &TargetNode::humanCallback, this);
		// Subscribe also to model states to calculate total distances
		modelStatesForDistance_sub = nh.subscribe("/gazebo/model_states", 1, &TargetNode::modelStatesForDistanceCallback, this);
	}

	else
		modelStates_sub = nh.subscribe("/gazebo/model_states", 1, &TargetNode::modelStatesCallback, this);
	odom_sub = nh.subscribe("base_pose_ground_truth", 1, &TargetNode::odomCallback, this);
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

	/*tf2_ros::TransformListener tf2_listener(tf_buffer);
	if (tf_buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(2.0)))
	{
		odom_to_map = tf_buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
		tf2::doTransform(odom_in_map_frame, odom_in_map_frame, odom_to_map);
	}
	*/
}

void TargetNode::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &costmap_msg)
{
	costmap = *costmap_msg;
}

void TargetNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states)
{
	int actor0_index = getIndex(model_states->name, "actor0");

	// Set the yaw angle from substracting consequent positions
	yaw = std::atan2((model_states->pose[actor0_index].position.y - actor_pose.pose.position.y),
					 (model_states->pose[actor0_index].position.x - actor_pose.pose.position.x));

	// translate yaw into a Quaternion
	tf::Quaternion q;
	q.setRPY(0.0, 0.0, yaw);

	// Convert quaternion to msg
	geometry_msgs::Quaternion q_msg;
	tf::quaternionTFToMsg(q, q_msg);

	// Set the orientation quaternion
	actor_pose.pose.orientation = q_msg;

	// Set the position
	actor_pose.pose.position = model_states->pose[actor0_index].position;

	actor_pose.header.stamp = ros::Time::now();
	actor_pose.header.frame_id = "map";
	followed_person_pub.publish(actor_pose);
	actor_message_arrived = true;
}

void TargetNode::modelStatesForDistanceCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states)
{
	int actor0_index = getIndex(model_states->name, "actor0");

	// Set the position
	actor_pose_ground_truth.pose.position = model_states->pose[actor0_index].position;
}

void TargetNode::peopleTrackedCallback(const leg_tracker::PersonArray::ConstPtr &person_array)
{
	// Set the position and orientation quaternion
	if (person_array->people.size() != 0)
		actor_pose.pose = person_array->people[0].pose;

	// Get the yaw angle from quaternion
	tf::Pose pose;
	tf::poseMsgToTF(actor_pose.pose, pose);
	yaw = tf::getYaw(pose.getRotation());

	actor_pose.header.stamp = ros::Time::now();
	actor_pose.header.frame_id = "map";
	followed_person_pub.publish(actor_pose);
	actor_message_arrived = true;
}

void TargetNode::humanCallback(const geometry_msgs::PoseStamped::ConstPtr &human_pos)
{
	// Set the position and orientation quaternion
	actor_pose.pose = human_pos->pose;

	// Get the yaw angle from quaternion
	tf::Pose pose;
	tf::poseMsgToTF(actor_pose.pose, pose);
	yaw = tf::getYaw(pose.getRotation());

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
		// Calculate average distance between the human and the robot
		double delta_x_ground_truth = actor_pose_ground_truth.pose.position.x - odom_in_map_frame.pose.position.x;
		double delta_y_ground_truth = actor_pose_ground_truth.pose.position.y - odom_in_map_frame.pose.position.y;
		total_distance += sqrt(pow(delta_y_ground_truth, 2) + pow(delta_x_ground_truth, 2));
		average_distance = total_distance / avg_count;
		cout << "average_distance: " << average_distance << endl;
		avg_count++;

		// Calculate traveled distance by the robot
		if (!first_time)
		{
			previous_x_robot = odom_in_map_frame.pose.position.x;
			previous_y_robot = odom_in_map_frame.pose.position.y;

			previous_x_human = actor_pose_ground_truth.pose.position.x;
			previous_y_human = actor_pose_ground_truth.pose.position.y;
			first_time = ros::Time::now().toSec();
		}
		else
		{
			double delta_x_robot = odom_in_map_frame.pose.position.x - previous_x_robot;
			double delta_y_robot = odom_in_map_frame.pose.position.y - previous_y_robot;
			total_distance_robot += sqrt(pow(delta_y_robot, 2) + pow(delta_x_robot, 2));
			cout << "total_distance_robot: " << total_distance_robot << endl;

			double delta_x_human = actor_pose_ground_truth.pose.position.x - previous_x_human;
			double delta_y_human = actor_pose_ground_truth.pose.position.y - previous_y_human;
			total_distance_human += sqrt(pow(delta_y_human, 2) + pow(delta_x_human, 2));
			cout << "total_distance_human: " << total_distance_human << endl;
			cout << "time: " << ros::Time::now().toSec() - first_time << endl
				 << endl;

			previous_x_robot = odom_in_map_frame.pose.position.x;
			previous_y_robot = odom_in_map_frame.pose.position.y;

			previous_x_human = actor_pose_ground_truth.pose.position.x;
			previous_y_human = actor_pose_ground_truth.pose.position.y;
		}
		// Set the orientation
		target.pose.orientation = actor_pose.pose.orientation;
		if (FOLLOW_MODE == "between the robot and human")
		{
			// Stay some distance behind the human and also between the robot and human
			double delta_x = actor_pose.pose.position.x - odom_in_map_frame.pose.position.x;
			double delta_y = actor_pose.pose.position.y - odom_in_map_frame.pose.position.y;
			// Set the difference vector angle using the difference vector
			double diff_angle = std::atan2(delta_y, delta_x);

			// If the costmap between the human and robot is occupied check the arc
			// that is 1m behind the human
			double delta_yaw = 0;
			int costmap_idx = 0;
			// translate yaw into a Quaternion
			tf::Quaternion q;

			for (int i = 0; i < 180; i++)
			{
				delta_yaw = PI / 180.0 * i;
				// Check plus delta_yaw
				// Normalize between -PI, +PI
				double normalized_angle = fmod((diff_angle + delta_yaw + PI), (2 * PI)) - PI;
				target.pose.position.x = actor_pose.pose.position.x -
										 DISTANCE_TO_FOLLOW_BEHIND * cos(normalized_angle);
				target.pose.position.y = actor_pose.pose.position.y -
										 DISTANCE_TO_FOLLOW_BEHIND * sin(normalized_angle);

				costmap_idx = mapToCostmapIdx(target.pose.position.x,
											  target.pose.position.y,
											  costmap.info.width,
											  costmap.info.height,
											  costmap.info.resolution);
				if ((int)costmap.data[costmap_idx] == 0)
				{
					q.setRPY(0.0, 0.0, diff_angle + delta_yaw);
					break;
				}

				// Check minus delta_yaw
				// Normalize between -PI, +PI
				normalized_angle = fmod((diff_angle - delta_yaw + PI), (2 * PI)) - PI;
				target.pose.position.x = actor_pose.pose.position.x -
										 DISTANCE_TO_FOLLOW_BEHIND * cos(normalized_angle);
				target.pose.position.y = actor_pose.pose.position.y -
										 DISTANCE_TO_FOLLOW_BEHIND * sin(normalized_angle);

				costmap_idx = mapToCostmapIdx(target.pose.position.x,
											  target.pose.position.y,
											  costmap.info.width,
											  costmap.info.height,
											  costmap.info.resolution);
				if ((int)costmap.data[costmap_idx] == 0)
				{
					q.setRPY(0.0, 0.0, diff_angle - delta_yaw);
					break;
				}
			}

			// Convert quaternion to msg
			geometry_msgs::Quaternion q_msg;
			tf::quaternionTFToMsg(q, q_msg);
			target.pose.orientation = q_msg;
		}
		else if (FOLLOW_MODE == "behind the human")
		{
			// Stay exactly behind the human by substracting in the direction where he looks
			target.pose.position.x = actor_pose.pose.position.x -
									 DISTANCE_TO_FOLLOW_BEHIND * cos(yaw);
			target.pose.position.y = actor_pose.pose.position.y -
									 DISTANCE_TO_FOLLOW_BEHIND * sin(yaw);
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
				// Normalize between -PI, +PI
				double normalized_angle = fmod((yaw + delta_yaw + PI), (2 * PI)) - PI;
				target.pose.position.x = actor_pose.pose.position.x -
										 DISTANCE_TO_FOLLOW_BEHIND * cos(normalized_angle);
				target.pose.position.y = actor_pose.pose.position.y -
										 DISTANCE_TO_FOLLOW_BEHIND * sin(normalized_angle);

				costmap_idx = mapToCostmapIdx(target.pose.position.x,
											  target.pose.position.y,
											  costmap.info.width,
											  costmap.info.height,
											  costmap.info.resolution);
				if ((int)costmap.data[costmap_idx] == 0)
					break;

				// Check minus delta_yaw
				// Normalize between -PI, +PI
				normalized_angle = fmod((yaw - delta_yaw + PI), (2 * PI)) - PI;
				target.pose.position.x = actor_pose.pose.position.x -
										 DISTANCE_TO_FOLLOW_BEHIND * cos(normalized_angle);
				target.pose.position.y = actor_pose.pose.position.y -
										 DISTANCE_TO_FOLLOW_BEHIND * sin(normalized_angle);

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
	ros::Rate loop_rate(5);

	while (ros::ok())
	{
		node.mainLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}