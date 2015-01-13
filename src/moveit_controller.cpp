#include <ros/ros.h>

// #include <iostream>
#include <vector>
#include <string>
// #include <fstream>
// #include <cmath>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/DisplayTrajectory.h>
// #include <moveit_msgs/DisplayRobotState.h>

//KDL Kinematics Plugin
//#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
// #include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
// #include <tf_conversions/tf_eigen.h>

// #include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>

#include <rose_moveit_controller/arm_goalAction.h>

class arm_controller
{
  protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<rose_moveit_controller::arm_goalAction> as_; 

  moveit::planning_interface::MoveGroup group;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 

  std::string action_name_;
  rose_moveit_controller::arm_goalFeedback feedback_;
  rose_moveit_controller::arm_goalResult result_;

  //For Computation -> TODO Parameterize it!
  std::string group_name;
  ros::WallTime s_ros, e_ros;

  bool visualize;
  bool CollisionCheck;
  bool execute;
  bool go_for_it;

  bool cartesian_control; 

  int best_index;
  
  tf::Quaternion q0;

  // High Level Waypoints
  geometry_msgs::Pose target_pose1;

  // ros::Publisher display_publisher;

public:

arm_controller(std::string name, std::string group_name_) :
  as_(nh_, name, boost::bind(&arm_controller::executeCB, this, _1), false),
  action_name_(name), group(group_name_)
{
	//Initialization Stuffs	
	visualize = true;
	CollisionCheck = false;
	execute = true;

	cartesian_control = false;

	go_for_it = true;

	group_name = group_name_;

	// display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	// moveit_msgs::DisplayTrajectory display_trajectory;

	ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());
	ROS_INFO("End Effector Link: %s", group.getEndEffectorLink().c_str());

	//Wait for other nodes to Come up! This is a Quick Hack, of course!
	sleep(6.0);

	//if(CollisionCheck) 
	//	addObstacles(group, planning_scene_interface);

	//Get Current Robot Pose
	geometry_msgs::PoseStamped cur_pose_;
	cur_pose_= group.getCurrentPose(group.getEndEffectorLink());

	//Here the Positions are approx. (0.0, 0.0, 0.53) and orientation (0.0, 0.0, 0.707, 0.707)
	ROS_INFO("CURRENT POSE of MECON ARM: Position(%f, %f, %f) Orientation (%f, %f, %f, %f)",
								cur_pose_.pose.position.x, 
								cur_pose_.pose.position.y, 
								cur_pose_.pose.position.z,
								cur_pose_.pose.orientation.x, 
								cur_pose_.pose.orientation.y, 
								cur_pose_.pose.orientation.z, 
								cur_pose_.pose.orientation.w);

	//Handy function to convert from RPY to Quaternion. 
	q0 = tf::createQuaternionFromRPY(1.57,0.0,0.0);
  
	//But we will now use it now. Lets copy the current orientation to q0. We will use this quaternion to constrain the gripper in the cartesian planner
	q0[0] = cur_pose_.pose.orientation.x;
	q0[1] = cur_pose_.pose.orientation.y;
	q0[2] = cur_pose_.pose.orientation.z;
	q0[3] = cur_pose_.pose.orientation.w;

	// Both current position and orientation are copied
	target_pose1 = cur_pose_.pose;

  	robot_state::RobotState start_state(*group.getCurrentState());
	group.setStartState(start_state);

	// Set the Panning time
	group.setPlanningTime(0.5);
	group.setGoalTolerance(0.005);

	// All Set.. Start The Server!!
	as_.start();
}

~arm_controller(void)
{
}

//TODO: Parameterize this!
void addObstacles(moveit::planning_interface::MoveGroup &group, moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, geometry_msgs::Pose obPose)
{
	planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames(false));
	moveit_msgs::CollisionObject collision_object,collision_object1,collision_object2,collision_object3;
	collision_object.header.frame_id = group.getPlanningFrame();
	/*collision_object1.header.frame_id = group.getPlanningFrame();
	collision_object2.header.frame_id = group.getPlanningFrame();
	collision_object3.header.frame_id = group.getPlanningFrame();*/

	collision_object.id = "box1";
	/*collision_object1.id = "box2";
	collision_object2.id = "box3";
	collision_object3.id = "box4";*/

	shape_msgs::SolidPrimitive primitive; //,primitive1,primitive2,primitive3;

	primitive.type = primitive.BOX;
	/*primitive1.type = primitive.BOX;
	primitive2.type = primitive.BOX;
	primitive3.type = primitive.BOX;*/

	primitive.dimensions.resize(3); 
	/*primitive1.dimensions.resize(3);
	primitive2.dimensions.resize(3);
	primitive3.dimensions.resize(3);*/

	primitive.dimensions[0] = 0.05;
	primitive.dimensions[1] = 0.4;
	primitive.dimensions[2] = 0.08;

	/*primitive1.dimensions[0] = 0.2;
	primitive1.dimensions[1] = 0.05;
	primitive1.dimensions[2] = 0.08;

	primitive2.dimensions[0] = 0.2;
	primitive2.dimensions[1] = 0.05;
	primitive2.dimensions[2] = 0.08;

	primitive3.dimensions[0] = 0.05;
	primitive3.dimensions[1] = 0.2;
	primitive3.dimensions[2] = 0.08;*/

	geometry_msgs::Pose box_pose; //,box_pose1,box_pose2,box_pose3;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  obPose.position.x - 0.04;
	box_pose.position.y =  obPose.position.y + 0.05;
	box_pose.position.z =  obPose.position.z - 0.02;


	/*box_pose1.orientation.w = 1.0;
	box_pose1.position.x =  obPose.position.x - 0.09;
	box_pose1.position.y =  obPose.position.y + 0.09;
	box_pose1.position.z =  obPose.position.z + 0.08;


	box_pose2.orientation.w = 1.0;
	box_pose2.position.x =  obPose.position.x + 0.09;
	box_pose2.position.y =  obPose.position.y - 0.09;
	box_pose2.position.z =  obPose.position.z + 0.08;

	box_pose3.orientation.w = 1.0;
	box_pose3.position.x =  obPose.position.x - 0.09;
	box_pose3.position.y =  obPose.position.y - 0.09;
	box_pose3.position.z =  obPose.position.z + 0.08;*/

	/*box_pose.position.x =  0.1; //obPose.position.x; //0.1;
	box_pose.position.y =  0.1; //obPose.position.y; // - 0.02; //-0.1;
	box_pose.position.z =  0.4; //obPose.position.z + 0.16; //0.4;*/

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	/*collision_object1.primitives.push_back(primitive1);
	collision_object1.primitive_poses.push_back(box_pose1);
	collision_object1.operation = collision_object.ADD;

	collision_object2.primitives.push_back(primitive2);
	collision_object2.primitive_poses.push_back(box_pose2);
	collision_object2.operation = collision_object.ADD;

	collision_object3.primitives.push_back(primitive3);
	collision_object3.primitive_poses.push_back(box_pose3);
	collision_object3.operation = collision_object.ADD;*/

	std::vector<moveit_msgs::CollisionObject> collision_objects;  
	collision_objects.push_back(collision_object);  
	/*collision_objects.push_back(collision_object1);  
	collision_objects.push_back(collision_object2);  
	collision_objects.push_back(collision_object3);  */

	ROS_INFO("Add an object into the world");  
	planning_scene_interface.addCollisionObjects(collision_objects);
  
	//sleep(2.0);
}


bool computeOMPLPlan(int plannerID, 
	geometry_msgs::Pose &target_pose1, 
	moveit::planning_interface::MoveGroup::Plan &my_plan, 
	moveit::planning_interface::MoveGroup &group)
{
	if(plannerID==0) group.setPlannerId("RRTkConfigDefault"); 
	else if(plannerID==1) group.setPlannerId("RRTConnectkConfigDefault");
	else if(plannerID==2) group.setPlannerId("SBLkConfigDefault");
	else if(plannerID==3) group.setPlannerId("ESTkConfigDefault");
	else if(plannerID==4) group.setPlannerId("LBKPIECEkConfigDefault");
	else if(plannerID==5) group.setPlannerId("BKPIECEkConfigDefault");
	else if(plannerID==6) group.setPlannerId("KPIECEkConfigDefault");
	else if(plannerID==7) group.setPlannerId("RRTstarkConfigDefault");
	else if(plannerID==8) group.setPlannerId("TRRTkConfigDefault");
	else if(plannerID==9) group.setPlannerId("PRMkConfigDefault"); 
	else group.setPlannerId("PRMstarkConfigDefault"); 

	// Display the Choice for FYI!
	ROS_INFO("------------------PLANNER: %d In Execution!------------------",plannerID);

	// Conversion of Quaternion to RPY
	tf::Quaternion q;
  	double roll, pitch, yaw;
	tf::quaternionMsgToTF(target_pose1.orientation, q);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		
	ROS_INFO("Trying to Move End Effector to Position [%.2f, %.2f, %.2f] and Orientation [%.2f, %.2f, %.2f]", 
			target_pose1.position.x, target_pose1.position.y, target_pose1.position.z, roll, pitch, yaw);

	// Set Group Target Pose to WP 1
	group.setPoseTarget(target_pose1); // This is being set again later. This is because OMPL planners need to have a target to plan!

	// Check if the Planner can Make a Plan
	return group.plan(my_plan); 
}

std::vector<geometry_msgs::Pose> constructWaypointList(moveit::planning_interface::MoveGroup::Plan &my_plan, 
		moveit::planning_interface::MoveGroup &group, tf::Quaternion q0, std::vector<geometry_msgs::Pose> &waypoints)
{
	ROS_INFO("Number of Points in Computed Trajectory: %ld", my_plan.trajectory_.joint_trajectory.points.size());

	int sampling_density = my_plan.trajectory_.joint_trajectory.points.size() * 0.2; //TODO TBD!! -> variable
	if(sampling_density < 1) sampling_density = 1;

        double joint[6]; //TODO: Configurable

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description"); 
	moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());  
	moveit::core::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

	kinematic_state->setToDefaultValues(); 
	const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
	ROS_INFO("Test: %s", group_name.c_str());  

	for(int sample_index = 1; sample_index < my_plan.trajectory_.joint_trajectory.points.size(); sample_index += sampling_density)
	{
		ROS_INFO("sample density: %d", sampling_density);

		joint[0] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[0];
		joint[1] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[1];
		joint[2] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[2];
		joint[3] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[3];
		joint[4] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[4];
		joint[5] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[5];
		//joint[6] = my_plan.trajectory_.joint_trajectory.points[sample_index].positions[6];
		
		//ROS_INFO("Sampled Joint Set: [%lf; %lf; %lf; %lf; %lf; %lf; %lf]", joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],joint[6]);
		ROS_INFO("Sampled Joint Set: [%lf; %lf; %lf; %lf; %lf; %lf]", joint[0],joint[1],joint[2],joint[3],joint[4],joint[5]);

		kinematic_state->setVariablePositions(joint);	

		// Finally, the Forward Kinematics!
		const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("gripper_link_1");
		geometry_msgs::Pose m;
		tf::poseEigenToMsg(end_effector_state, m);
		m.orientation.x = q0[0];
		m.orientation.y = q0[1];
		m.orientation.z = q0[2];
		m.orientation.z = q0[3];
		
		// Construct the Waypoint Vector
		waypoints.push_back(m);		
	}

	ROS_INFO("Constructed Waypoint Vector, Waypoint Vector Size: %ld", waypoints.size());

	// Set the group pose target as the last point in the waypoint vector => Same as start state in this case.
	group.setPoseTarget(waypoints.back());

	return waypoints;
}

// THE CORE!!
void executeCB(const rose_moveit_controller::arm_goalGoalConstPtr &goal)
{
	// helper variables
	ros::Rate r(1);
	bool success_ = true;

	float vx = goal->vx;
	float vy = goal->vy;
	float vz = goal->vz;

	// We have here: feedback_., result_., goal->!!! 
	s_ros = ros::WallTime::now();

	ROS_INFO("Going to Plan to GOAL...");  

	// Make a Plan Objects
	moveit::planning_interface::MoveGroup::Plan my_plan;
	moveit::planning_interface::MoveGroup::Plan my_plan_[11];
	bool res[11];

	ROS_INFO("---------------------------------- OMPL JOINT SPACE PLANNING -------------------------------------------");

	target_pose1.position.x = goal->goal_pose.pose.position.x;
	target_pose1.position.y = goal->goal_pose.pose.position.y;
	target_pose1.position.z = goal->goal_pose.pose.position.z;
	target_pose1.orientation.x = goal->goal_pose.pose.orientation.x;
	target_pose1.orientation.y = goal->goal_pose.pose.orientation.y;
	target_pose1.orientation.z = goal->goal_pose.pose.orientation.z;
	target_pose1.orientation.w = goal->goal_pose.pose.orientation.w;

	// One way of setting group to current state ==> Get it from the robot!
	//robot_state::RobotState start_state(*group.getCurrentState());
	//group.setStartState(start_state);

	if(goal->control_mode == 0) cartesian_control = true; else cartesian_control = false;

	// Set the Panning time
	group.setPlanningTime(0.5);
	group.setGoalTolerance(0.005);

	// Just add an obstacle -> when the goal message says so
	if(goal->addObstacle == 1)
		addObstacles(group, planning_scene_interface, goal->goal_pose.pose);
	else
		planning_scene_interface.removeCollisionObjects(planning_scene_interface.getKnownObjectNames(false));

	//ROS_INFO("Going to Wait 10 Seconds for the Environement to Update....");
	//sleep(10);

	// Another way of setting group to current state
	//group.setStartStateToCurrentState();
  	robot_state::RobotState start_state(*group.getCurrentState());
	group.setStartState(start_state);

	s_ros = ros::WallTime::now();

	// Another way of setting group to current state
	//group.setStartStateToCurrentState();

	feedback_.feedback = "Going to Try First Best Planner";
	as_.publishFeedback(feedback_);

	best_index = 0;

	if(cartesian_control)
	{
		ROS_INFO("==================== START of CARTESIAN CONTROL LOOP ====================");
		// compute plans from planners
		for(int i = 0; i <= 10; i++)
		{
			my_plan_[i].trajectory_.joint_trajectory.points.clear();
			my_plan.trajectory_.joint_trajectory.points.clear();

			if(computeOMPLPlan(i, target_pose1, my_plan, group))
			{
				res[i] = true;
				my_plan_[i] = my_plan;
				best_index = i;
			}
			else
			{
				res[i] = false;
			}

			if(res[best_index]) break;
		}

		for(int i = 0; i <= best_index; i++)
			ROS_INFO("Planner_ID: %d, Success: %d, Trajectory_Size: %ld", 
					i, res[i], my_plan_[i].trajectory_.joint_trajectory.points.size());

		feedback_.feedback = "Plans Have been Created. Going to Execute..";
		as_.publishFeedback(feedback_);

		if(res[best_index])
		{
			int indx = best_index;
			// The trajectory needs to be modified so it will include velocities as well.
			// First to create a RobotTrajectory object
			robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group_name);

			// Second get a RobotTrajectory from trajectory
			rt.setRobotTrajectoryMsg(*group.getCurrentState(), my_plan_[indx].trajectory_);

			// Thrid create a IterativeParabolicTimeParameterization object
			trajectory_processing::IterativeParabolicTimeParameterization iptp;

			// Fourth compute computeTimeStamps
			bool success_ = iptp.computeTimeStamps(rt);
			ROS_INFO("Computed time stamp %s",success_?"SUCCEDED":"FAILED");

			// Get RobotTrajectory_msg from RobotTrajectory
			rt.getRobotTrajectoryMsg(my_plan_[indx].trajectory_);

			// Check trajectory_msg for velocities not empty
			//std::cout << trajectory << std::endl;
			moveit::planning_interface::MoveGroup::Plan plan;
			plan.trajectory_ = my_plan_[indx].trajectory_;

			//ROS_INFO("Visualizing Cartesian Path (%.2f%% acheived)", fraction * 100.0);   
			//sleep(5.0);

		  	group.execute(plan);

			feedback_.feedback = "Execution Complete. Server going to idle.";			
			as_.publishFeedback(feedback_);

			result_.result = 1.0;
		}
		else
		{
			ROS_INFO("Sorry!! Didint Manage to find a feasible Path..");
			feedback_.feedback = "No Feasible Solution was Found by the planners!!";			
			as_.publishFeedback(feedback_);

			result_.result = 0.0;
		}

		ROS_INFO("==================== END of CARTESIAN CONTROL LOOP ====================");

	}
	else
	{
		geometry_msgs::PoseStamped cur_pose_mecon;

		ros::Time begin = ros::Time::now();
		ros::Time end = ros::Time::now();
		ros::Time begin_ = ros::Time::now();
		ros::Time end_ = ros::Time::now();

		//for(int k = 0; k < 10; k++)
		while(!as_.isPreemptRequested())
		{	
			ROS_INFO("==================== START of VELOCITY CONTROL LOOP ====================");

			//if(as_.isNewGoalAvailable()) 
			//	ROS_INFO("--------------------------------New GOAL AVAILABLE!!!-------------------------");

			begin = ros::Time::now();

			cur_pose_mecon = group.getCurrentPose(group.getEndEffectorLink());
			ROS_INFO("CURRENT POSE of MECON ARM: Position(%f, %f, %f) Orientation (%f, %f, %f, %f)",
									cur_pose_mecon.pose.position.x, 
									cur_pose_mecon.pose.position.y, 
									cur_pose_mecon.pose.position.z,
									cur_pose_mecon.pose.orientation.x, 
									cur_pose_mecon.pose.orientation.y, 
									cur_pose_mecon.pose.orientation.z, 
									cur_pose_mecon.pose.orientation.w);
			target_pose1 = cur_pose_mecon.pose;

			if(vx > 0.0) target_pose1.position.x += 0.01;
			else if(vx < 0.0) target_pose1.position.x -= 0.01;
			else target_pose1.position.x = target_pose1.position.x;

			if(vy > 0.0) target_pose1.position.y += 0.01;
			else if(vy < 0.0) target_pose1.position.y -= 0.01;
			else target_pose1.position.y = target_pose1.position.y;


			if(vz > 0.0) target_pose1.position.z += 0.01;
			else if(vz < 0.0) target_pose1.position.z -= 0.01;
			else target_pose1.position.z = target_pose1.position.z;

		  	robot_state::RobotState start_state(*group.getCurrentState());
			group.setStartState(start_state);

			best_index = 0;

			// Conversion of Quaternion to RPY
			tf::Quaternion q;
		  	double roll, pitch, yaw;
			tf::quaternionMsgToTF(target_pose1.orientation, q);
			tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
		
			ROS_INFO("Move: End Effector to Pose [%.2f, %.2f, %.2f] and Orientation [%.2f, %.2f, %.2f]", 
					target_pose1.position.x, target_pose1.position.y, target_pose1.position.z, roll, pitch, yaw);


			for(int i = 0; i <= 10; i++)
			{
				my_plan_[i].trajectory_.joint_trajectory.points.clear();
				my_plan.trajectory_.joint_trajectory.points.clear();

				if(computeOMPLPlan(i, target_pose1, my_plan, group))
				{
					res[i] = true;
					my_plan_[i] = my_plan;
					best_index = i;
				}
				else
				{
					res[i] = false;
				}

				if(res[best_index]) break;
			}

			for(int i = 0; i <= best_index; i++)
				ROS_INFO("Planner_ID: %d, Success: %d, Trajectory_Size: %ld", 
						i, res[i], my_plan_[i].trajectory_.joint_trajectory.points.size());

			feedback_.feedback = "Plans Have been Created. Going to Execute..";
			as_.publishFeedback(feedback_);


			if(res[best_index])
			{
				int indx = best_index;
				// The trajectory needs to be modified so it will include velocities as well.
				// First to create a RobotTrajectory object
				robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), group_name);

				// Second get a RobotTrajectory from trajectory
				rt.setRobotTrajectoryMsg(*group.getCurrentState(), my_plan_[indx].trajectory_);

				// Thrid create a IterativeParabolicTimeParameterization object
				trajectory_processing::IterativeParabolicTimeParameterization iptp;

				// Fourth compute computeTimeStamps
				bool success_ = iptp.computeTimeStamps(rt);
				ROS_INFO("Computed time stamp %s",success_?"SUCCEDED":"FAILED");

				// Get RobotTrajectory_msg from RobotTrajectory
				rt.getRobotTrajectoryMsg(my_plan_[indx].trajectory_);

				////////////////// START OF VARIABLE VELOCITY CONTROL //////////////////////////////////////////////
				ROS_INFO("Processing Trajectory: Scaling Velocities");
				for(int l = 0; l < my_plan_[indx].trajectory_.joint_trajectory.points.size(); l++)
				{					
					 my_plan_[indx].trajectory_.joint_trajectory.points[l].time_from_start = 
						my_plan_[indx].trajectory_.joint_trajectory.points[l].time_from_start; //* 0.01;
				
					for(int m = 0; m < 6; m++)
					{
						my_plan_[indx].trajectory_.joint_trajectory.points[l].velocities[m] =
							my_plan_[indx].trajectory_.joint_trajectory.points[l].velocities[m]; //* 100.0;
						my_plan_[indx].trajectory_.joint_trajectory.points[l].accelerations[m] =
							my_plan_[indx].trajectory_.joint_trajectory.points[l].accelerations[m]; //* 100.0;
						my_plan_[indx].trajectory_.joint_trajectory.points[l].positions[m] =
							my_plan_[indx].trajectory_.joint_trajectory.points[l].positions[m];
					}
				}
				////////////////// END OF VARIABLE VELOCITY CONTROL //////////////////////////////////////////////

				// Check trajectory_msg for velocities not empty
				//std::cout << my_plan_[indx].trajectory_ << std::endl;

				moveit::planning_interface::MoveGroup::Plan plan;
				plan.trajectory_ = my_plan_[indx].trajectory_;

				//ROS_INFO("Visualizing Cartesian Path (%.2f%% acheived)", fraction * 100.0);   
				//sleep(5.0);

				begin_ = ros::Time::now();

			  	group.execute(plan);

				end_ = ros::Time::now();

				feedback_.feedback = "Execution Complete. Server going to idle.";			
				as_.publishFeedback(feedback_);
				
				end = ros::Time::now();

				result_.result = 1.0;
				ROS_INFO("Total Computation & Execution Loop Time Taken: %f",(end - begin).toSec());
				ROS_INFO("Computation Time Taken: %f",(end - begin).toSec() - (end_ - begin_).toSec());
				ROS_INFO("Execution Time Taken: %f",(end_ - begin_).toSec());

				ROS_INFO("==================== DONE VELOCITY CONTROL LOOP ====================");
			}
			else
			{
				ROS_INFO("Sorry!! Didint Manage to find a feasible Path..");
				feedback_.feedback = "No Feasible Solution was Found by the planners!!";			
				as_.publishFeedback(feedback_);
				result_.result = 0.0;
			}
		}
	}

	//if(go_for_it)
	//result_.result = (e_ros - s_ros).toSec();
	as_.setSucceeded(result_);

}// End of Function ExecuteCB

}; //End of Class Definition

// Finally, the main..
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rose_moveit_server");

  arm_controller ac(ros::this_node::getName(),"jaco_arm_moveit_server");
  ros::spin();

  return 0;
}
