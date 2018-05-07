/**
    close_and_record.cpp

    Purpose: a ROS node for closing the hand and recording on a bag file the norm and the components 
    of accelerometer readings of Pisa IIT SoftHand fingers using the IMU Glove.

    N.B: 	- The qb_interface_imu node from package qb_interface must be running. 
            - The norm_publisher node from this package must be running. 
    		- The ids of the imus should be sequential from 0 to NUM_IMUS - 1.

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float64MultiArray.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <std_msgs/Int8.h>

// ROS CUSTOM MSGS
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

// OTHER INCLUDES
#include <math.h> 

// DEFINES
#define DEBUG       	0           	// Prints additional outputs if 1
#define NUM_IMUS 		16				// Number of available imus

#define CLOSE_TIME		4000			// Time in which the SoftHand closes totally
#define N_WP_CLOSE		150 			// Number of trajectory points of the slow hand closing
#define SKIP_TRAJ_DELAY	50              // Delay in ms to avoid "first trajectory before current time" in first hand close
#define HAND_TIMEOUT    7.0             // Hand closing timeout after which, if no collision, go on

#define RECORD_TOPIC_1	"/norm_publishing_topic"
#define RECORD_TOPIC_2	"/qb_class_imu/acc"
#define HAND_NAME		"right_hand"
#define HAND_JOINT 		"right_hand_synergy_joint"


// GLOBAL VARIABLES

// Action client ptr for hand
std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> move_;

// Subscriber for handle finger collision
ros::Subscriber finger_col_sub;

int touching_finger = 0;                // For gradual closing and arm stopping


//-----------------------------------------------------------------------------------------------------------------------//
// AUXILIARY FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------//

/* ******************************************************************************************* */
trajectory_msgs::JointTrajectoryPoint gen_point(double position, double velocity, ros::Duration t){
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(position);
    p.velocities.push_back(velocity);
    p.time_from_start = t;
    return p;
}

/* ******************************************************************************************* */
control_msgs::FollowJointTrajectoryGoal create_trajectory_goal(double position){
    auto nanosecond = ros::Duration(0, 1);
    auto millisecond = ros::Duration(0, 1000000);
    
    // Plan trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back(HAND_JOINT);
    // Delay in ms to avoid "first trajectory before current time" in first hand close
    traj.header.stamp = ros::Time::now() + ros::Duration(0, int (SKIP_TRAJ_DELAY) * 1000000);

    // Pushing back trajectory points to slow down the hand closing
    int time_increment = floor(CLOSE_TIME / N_WP_CLOSE);

    std::cout << "References given to hand are: " << std::endl;

    for(int i = 1; i <= N_WP_CLOSE; i++){
    	double hand_frac = (double (i)) / (double (N_WP_CLOSE));
    	// velocity has to be 0!!!!!!!!!!!!
    	traj.points.push_back(gen_point(position * hand_frac, 0.0, 
    		millisecond * (CLOSE_TIME - (N_WP_CLOSE - i) * time_increment) + nanosecond));

        std::cout << position * hand_frac << std::endl;
    }

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = traj;
    return goal;
}

/* ******************************************************************************************* */
void send_goal(double position){
    // Check whether the server is available
    if (move_->waitForServer(ros::Duration(1,0))){
    	// Create goal and send it to the controller
      	control_msgs::FollowJointTrajectoryGoal goal = create_trajectory_goal(position);
      
      	move_->sendGoal(goal);
    } else {
      	ROS_WARN_STREAM("No softhand server available. Are you running in simulation without Gazebo?");
    }
}

/* ******************************************************************************************* */
void sendHandTrajectory(double increment){
  	if (increment > 1.0) increment = 1.0;
  	else if (increment < 0.0) increment = 0.0;

  	send_goal(increment);
}

/* ******************************************************************************************** */
void fingerColCallback(const std_msgs::Int8::ConstPtr& msg){
    touching_finger = msg->data;
}

//-----------------------------------------------------------------------------------------------------------------------//
// MAIN
//-----------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv){
	// Initializing ROS node
	ros::init(argc, argv, "close_and_record");
	ros::NodeHandle cr_nh;

	// Initializing action client for hand
	move_ = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("/" + std::string(HAND_NAME) + "/joint_trajectory_controller/follow_joint_trajectory", true);

    // Subscribing to handle finger collision
    finger_col_sub = cr_nh.subscribe("/touching_finger_topic", 1000, fingerColCallback);

	// Closing hand slowly
	sendHandTrajectory(double (1.0));

    // sleep(5);

    // FOR TRYING TO CANCEL GOAL

    // Starting to measure time
    ros::Time before_time = ros::Time::now();
    ros::Duration timeout(HAND_TIMEOUT);

    bool first_print_out = true;
    ROS_INFO("I'm waiting for a finger collision!\n");

    while(touching_finger == 0 && ros::Time::now() - before_time < timeout){

        ros::spinOnce();
        
        if (!touching_finger) {
            if(first_print_out){
                ROS_INFO("No finger collision received yet!\n");
                first_print_out = false;
            }
        } else {
            ROS_INFO("A COLLISION DETECTED ON FINGER %d!\n", touching_finger);
            move_->cancelGoal();
        }
    }

	// While closing, record bag file
	ROS_INFO_STREAM("Starting to record the bag! \n");
	if(!move_->waitForResult(ros::Duration(20, 0))){
		std::cout << "The hand is not closing properly!" << std::endl;
	};

    sleep(1);

    // Closing hand completely
    sendHandTrajectory(double (1.0));

    sleep(3);

	// Opening hand
	sendHandTrajectory(double (0.0));

	// Success message
	std::cout << "The hand is reopened!" << std::endl;

	// Spin
	// ros::spin();
}