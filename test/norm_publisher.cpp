/**
    norm_publisher.cpp

    Purpose: a ROS node for publishing the norm of accelerometer readings of Pisa IIT SoftHand fingers using
    the IMU Glove.

    N.B: 	- The qb_interface_imu node from package qb_interface must be running. 
    		- The ids of the imus should be sequential from 0 to NUM_IMUS - 1.

    Input Topic: 	/qb_class_imu/acc
    Output Topic: 	/norm_publishing_topic

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

// ROS CUSTOM MSGS
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>

// OTHER INCLUDES
#include <math.h> 

// DEFINES
#define DEBUG       0           	// Prints additional outputs if 1
#define NUM_IMUS 	11				// Number of available imus (UPDATE THIS)!

std::vector<int> ids = {0, 1, 6, 7, 9, 10, 12, 13, 15, 16, 0}; 	// IDs of available imus (UPDATE THIS)!
// std::vector<int> ids = {0, 1, 6, 9, 10, 12, 13, 15, 16, 0}; 	// IDs of available imus (UPDATE THIS)!

std::string INPUT_TOPIC;
std::string OUTPUT_TOPIC;

// GLOBAL VARIABLES
ros::Publisher pub_norms;									// Publishes finger_id to OUTPUT_TOPIC 

// CURRENT MEASUREMENT
/* Current measurement from the read sensor */
float current_meas = 0.0;

//-----------------------------------------------------------------------------------------------------------------------//
// AUXILIARY FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------//

/* Gets from the parameter server the values of the needed vars loaded from config_norm.yaml */
bool getParamsOfYaml(){
	bool success = true;

	if(!ros::param::get("/norm_publisher/INPUT_TOPIC", INPUT_TOPIC)){
		ROS_WARN("INPUT_TOPIC param not found in param server! Using default.");
		INPUT_TOPIC = "/qb_class_imu/acc";
		success = false;
	}
	if(!ros::param::get("/norm_publisher/OUTPUT_TOPIC", OUTPUT_TOPIC)){
		ROS_WARN("OUTPUT_TOPIC param not found in param server! Using default.");
		OUTPUT_TOPIC = "/norm_publishing_topic";
		success = false;
	}

	return success;
}

/* Gets in input a qb_interface::inertialSensor message and returns the norm of the measurement */
float getNormFromMeas(const qb_interface::inertialSensor meas){
	return sqrt(pow(meas.x, 2) + pow(meas.y, 2) + pow(meas.z, 2));
}

/* Gets in input a qb_interface::inertialSensorArray message and writes the of the inertialSensorArray 
	corresponding to the index specified by input_imu_array */
float writeMeasFromArray(const qb_interface::inertialSensorArray meas_array, const int input_imu_array){
	for(int i = 0; i < NUM_IMUS; i++){
		if(meas_array.m[i].id == input_imu_array){
			return getNormFromMeas(meas_array.m[i]);
		}
	}

	ROS_FATAL_STREAM("The specified IMU of id " << input_imu_array << " not found! Plaese double check the ids!");
	exit (EXIT_FAILURE);
}

//-----------------------------------------------------------------------------------------------------------------------//
//CALLBACK FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------//
/* Callback for IMU topic subscriber */
void normPublish(const qb_interface::inertialSensorArrayConstPtr& input_sensor_array){
	// Creating the message and filling it up
	std_msgs::Float64MultiArray pub_msg;

	for(auto i : ids){
		current_meas = writeMeasFromArray(*input_sensor_array, i);
		pub_msg.data.push_back(current_meas);
	}

	// Publish the current norm msg
	pub_norms.publish(pub_msg);

	// Couting the finger in collision
	if(DEBUG) std::cout << "The norm Callback done sucessfully!!!!" << std::endl;
}


//-----------------------------------------------------------------------------------------------------------------------//
// MAIN
//-----------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "norm_publisher");
	ros::NodeHandle np_nh;

	// Getting params from parameter server
	if(!getParamsOfYaml()) ROS_WARN("Some params not loaded correctly!");

	// Creating a ROS publisher for the output for publishing the finger_id of the finger in collision
	pub_norms = np_nh.advertise<std_msgs::Float64MultiArray>(std::string(OUTPUT_TOPIC), 10);

	// Creating a ROS subscriber for the input imu acceleration topic
	ros::Subscriber imu_sub = np_nh.subscribe(std::string(INPUT_TOPIC), 10, normPublish);

	// Success message
	std::cout << "The norms of the imus are being published!" << std::endl;

	// Spin
	ros::spin();

}