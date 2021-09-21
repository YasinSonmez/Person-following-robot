#include <target_generation_node.h>
using namespace std;
TargetNode::TargetNode()
{
	//ros::NodeHandle pn("~");

	// Subscribers
	odom_sub = nh.subscribe("/RosAria/odom", 10, &TargetNode::odomCallback, this);
	modelStates_sub = nh.subscribe("/gazebo/model_states", 10, &TargetNode::modelStatesCallback, this);

	// Publishers
	target_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	followed_person_pub = nh.advertise<geometry_msgs::PoseStamped>("/followed_person", 10);
}
int getIndex(std::vector<std::string> v, std::string value)
{
	for (int i = 0; i < v.size(); i++)
	{
		if (v[i].compare(value) == 0)
			return i;
	}
	return -1;
}

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

void TargetNode::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &model_states)
{
	int actor1_index = getIndex(model_states->name, "actor1");
	actor_pose.pose = model_states->pose[actor1_index];
	actor_pose.header.stamp = ros::Time::now();
	actor_pose.header.frame_id = "map";
	followed_person_pub.publish(actor_pose);
}

void TargetNode::mainLoop()
{
	// Stay some distance behind between the robot and actor
	double delta_x = actor_pose.pose.position.x - odom_in_map_frame.pose.position.x;
	double delta_y = actor_pose.pose.position.y - odom_in_map_frame.pose.position.y;
	double delta_r = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));
	// Distance that is substrated from the position of the actor
	double distance_to_follow_behind = 1.5;

	/*target.pose.position.x = actor_pose.pose.position.x -
							 distance_to_follow_behind * delta_x / delta_r;
	target.pose.position.y = actor_pose.pose.position.y -
							 distance_to_follow_behind * delta_y / delta_r;
	target.pose.position.z = 0;*/

	// Get orientation from quaternion
	tf::Quaternion q(
		actor_pose.pose.orientation.x,
		actor_pose.pose.orientation.y,
		actor_pose.pose.orientation.z,
		actor_pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	target.pose.orientation = tf::createQuaternionMsgFromYaw(yaw - PI / 2.0);

	target.pose.position.x = actor_pose.pose.position.x -
							 distance_to_follow_behind * cos(yaw - PI / 2.0);
	target.pose.position.y = actor_pose.pose.position.y -
							 distance_to_follow_behind * sin(yaw - PI / 2.0);
	target.pose.position.z = 0;

	// Header info
	target.header.stamp = ros::Time::now();
	target.header.frame_id = "map";
	target_pub.publish(target);
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "target_generation");

	TargetNode node;
	ros::Rate loop_rate(1);

	while (ros::ok())
	{
		node.mainLoop();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}