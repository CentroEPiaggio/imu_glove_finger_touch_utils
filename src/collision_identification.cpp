/**
    collision_identification.cpp

    Purpose: a ROS node for identification of collision on Pisa IIT SoftHand fingers using
    the IMU Glove.

    N.B: 	The qb_interface_imu node from package qb_interface must be running.

    N.B: 	As of now we are only using the imus of the distal link of each finger for identifying
    		collisions. In a later release all the imus may be used.

    Input Topic: 	/qb_class_imu/acc
    Output Topic: 	/touching_finger_topic

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>
#include <std_msgs/Int8.h>

// ROS CUSTOM MSGS
#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include <sensor_msgs/JointState.h>

// OTHER INCLUDES
#include "peak_identification.h"
#include <math.h> 

// DEFINES
#define DEBUG       0           	// Prints additional outputs if 1
#define USE_SUM		true			// Use sum of imus on finger instead of chosing biggest

std::string INPUT_TOPIC;
std::string OUTPUT_TOPIC;
std::string SYN_TOPIC;
std::string HAND_JOINT;

const int NUM_FINGERS = 5;			// Number of fingers of SoftHand
const int NUM_LAG = 40;				// Width of lag window of z score
double THRESHOLD;					// If z score is above this threshold a peak is identified
double INFLUENCE;					// Importance to give to a previous peak in the computation of the mean
double FINGER_TIMEOUT;				// Finger Timeout for resetting finger_id
float REF_THRESH;					// Minimum absolute treshold on imu acc. for identification
float ABS_MAX_THRESH;				// Maximum absolute treshold on imu acc. for identification (NOT USED FOR NOW)
float SYN_POS_THRESH;				// Threshold on synergy after which start detection 
float SYN_POS_THRESH_2;				// Threshold on synergy after which no detection 

// (NOT USED FOR NOW)
float SYN_VEL_WEIGHT;				// Weight on inverse of synergy velocity
float SYN_VEL_SAT_MIN;				// Value under which synergy velocity is made saturate
float SYN_VEL_SAT_MAX;				// Value over which synergy velocity is made saturate

/* Defining weights for each finger measurement as they move differently */
#define THUMB_WEIGHT		1.0		//1.0
#define INDEX_WEIGHT		1.0		//1.0
#define MIDDLE_WEIGHT		1.0		//0.99
#define RING_WEIGHT			1.0		//0.99
#define LITTLE_WEIGHT		1.0		//0.98
#define REF_WEIGHT			1.0		//1.00

// GLOBAL CONSTANTS
/* Arrays with ids of used sensors of each finger and reference imus */
const int thumb_imus[1] = {14};
const int index_imus[1] = {11};
const int middle_imus[1] = {8};
const int ring_imus[1] = {5};
const int little_imus[1] = {2};
const int ref_imus[1] = {15};

// GLOBAL VARIABLES
int finger_id = 0;											// Id of finger in collision (0 if none)
int biggest_finger_id = 0;									// Id of finger with biggest current acceleration
int old_finger_id = 0;										// Id of previous finger in collision (0 if none)
int num_saved_meas = 0;										// Number of saved measurements (for all fingers): should be NUM_LAG + 1

float synergy_pos = 0.0;									// Current hand synergy position value
float synergy_vel = 0.0;									// Current hand synergy velocity value
float synergy_weight = 0.0;									// Weight to be given to inputs according to inverse of synergy velocity

std_msgs::Int8 finger_id_msg;								// The message to be published using pub_fing_id
ros::Time finger_time;										// Time to be set when finger recieves a new collision
ros::Publisher pub_fing_id;									// Publishes finger_id to OUTPUT_TOPIC 

// Bools for checking absolute threshold on residuals
bool abs_thresh_1;
bool abs_thresh_2;
bool abs_thresh_3;
bool abs_thresh_4;
bool abs_thresh_5;
// Bool for checking reference imu
bool ref_thresh;
// Bool for checking if synergy position is beyond threshold
bool syn_thresh_ok;
// Bool for checking if all fingers have peaks (NOT USED FOR NOW)
bool all_fingers_peak;

// Bools for printing id hand is open or closed
bool first_below = true;
bool first_above = true;

// INPUT VECTORS
/* One vector for each finger and each vector has NUM_LAG previous measurements + 1 new measurement */
std::vector<ld>	thumb_input(NUM_LAG + 1);
std::vector<ld>	index_input(NUM_LAG + 1);
std::vector<ld>	middle_input(NUM_LAG + 1);
std::vector<ld>	ring_input(NUM_LAG + 1);
std::vector<ld>	little_input(NUM_LAG + 1);
std::vector<ld>	ref_input(NUM_LAG + 1);


// CURRENT MEASUREMENT
/* Current biggest measurement from the sensors of each finger (one per each finger) */
float thumb_meas = 0.0;
float index_meas = 0.0;
float middle_meas = 0.0;
float ring_meas = 0.0;
float little_meas = 0.0;
float ref_meas = 0.0;

// RESULT VARIABLES
std::unordered_map<std::string, std::vector<ld>> thumb_output;
std::unordered_map<std::string, std::vector<ld>> index_output;
std::unordered_map<std::string, std::vector<ld>> middle_output;
std::unordered_map<std::string, std::vector<ld>> ring_output;
std::unordered_map<std::string, std::vector<ld>> little_output;
std::unordered_map<std::string, std::vector<ld>> ref_output;

//-----------------------------------------------------------------------------------------------------------------------//
// AUXILIARY FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------//

/* Gets from the parameter server the values of the needed vars loaded from config_col.yaml */
bool getParamsOfYaml(){
	bool success = true;

	if(!ros::param::get("/finger_collision_identification/INPUT_TOPIC", INPUT_TOPIC)){
		ROS_WARN("INPUT_TOPIC param not found in param server! Using default.");
		INPUT_TOPIC = "/qb_class_imu/acc";
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/OUTPUT_TOPIC", OUTPUT_TOPIC)){
		ROS_WARN("OUTPUT_TOPIC param not found in param server! Using default.");
		OUTPUT_TOPIC = "/touching_finger_topic";
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/SYN_TOPIC", SYN_TOPIC)){
		ROS_WARN("SYN_TOPIC param not found in param server! Using default.");
		SYN_TOPIC = "/joint_states";
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/HAND_JOINT", HAND_JOINT)){
		ROS_WARN("HAND_JOINT param not found in param server! Using default.");
		HAND_JOINT = "right_hand_synergy_joint";
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/THRESHOLD", THRESHOLD)){
		ROS_WARN("THRESHOLD param not found in param server! Using default.");
		THRESHOLD = 7.0;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/INFLUENCE", INFLUENCE)){
		ROS_WARN("INFLUENCE param not found in param server! Using default.");
		INFLUENCE = 0.0;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/FINGER_TIMEOUT", FINGER_TIMEOUT)){
		ROS_WARN("FINGER_TIMEOUT param not found in param server! Using default.");
		FINGER_TIMEOUT = 5.0;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/REF_THRESH", REF_THRESH)){
		ROS_WARN("REF_THRESH param not found in param server! Using default.");
		REF_THRESH = 0.5;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/ABS_MAX_THRESH", ABS_MAX_THRESH)){
		ROS_WARN("ABS_MAX_THRESH param not found in param server! Using default.");
		ABS_MAX_THRESH = 0.50;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/SYN_POS_THRESH", SYN_POS_THRESH)){
		ROS_WARN("SYN_POS_THRESH param not found in param server! Using default.");
		SYN_POS_THRESH = 0.20;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/SYN_POS_THRESH_2", SYN_POS_THRESH_2)){
		ROS_WARN("SYN_POS_THRESH_2 param not found in param server! Using default.");
		SYN_POS_THRESH_2 = 0.70;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/SYN_VEL_WEIGHT", SYN_VEL_WEIGHT)){
		ROS_WARN("SYN_VEL_WEIGHT param not found in param server! Using default.");
		SYN_VEL_WEIGHT = 0.500;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/SYN_VEL_SAT_MIN", SYN_VEL_SAT_MIN)){
		ROS_WARN("SYN_VEL_SAT_MIN param not found in param server! Using default.");
		SYN_VEL_SAT_MIN = 0.02;
		success = false;
	}
	if(!ros::param::get("/finger_collision_identification/SYN_VEL_SAT_MAX", SYN_VEL_SAT_MAX)){
		ROS_WARN("SYN_VEL_SAT_MAX param not found in param server! Using default.");
		SYN_VEL_SAT_MAX = 0.50;
		success = false;
	}

	return success;
}

/* Gets in input a qb_interface::inertialSensor message and returns z component of the acceleration */
float getZFromMeas(const qb_interface::inertialSensor meas){
	return (meas.z);
	//return std::norm(meas.z);
}

/* Gets in input a qb_interface::inertialSensor message and returns the norm of the acceleration */
float getNormFromMeas(const qb_interface::inertialSensor meas){
	return sqrt(pow(meas.x, 2) + pow(meas.y, 2) + pow(meas.z, 2));
}

/* Gets in input a qb_interface::inertialSensorArray message and writes the relevant measurement between
the elements in the inertialSensorArray corresponding to the indexes specified by input_imu_array */
float writeMeasFromArray(const qb_interface::inertialSensorArray meas_array, const int input_imu_array[],
	const int size_array){
	if(DEBUG) std::cout << "ENTERED WRITE MEAS!!!!!!" << std::endl;
	int num_imus = size_array;
	if(DEBUG) std::cout << "GOT INPUT_IMU_ARRAY!!!!!! IT IS " << input_imu_array << std::endl;
	if(DEBUG) std::cout << "GOT INPUT_IMU_ARRAY SIZE WRITE MEAS!!!!!! SIZE IS " << num_imus << std::endl;
	float norm_result = 0;
	// In a loop, among the elements of meas_array identified by input_imu_array, find the biggest
	// or the sum (As we are using only 1 IMU per finger this does not matter)
	if(!USE_SUM){
		for(int i = 0; i < num_imus; i++){
			float curr_norm = getNormFromMeas(meas_array.m[input_imu_array[i]]); 
			// float curr_norm = getZFromMeas(meas_array.m[input_imu_array[i]]); 
			if(curr_norm > norm_result) norm_result = curr_norm;
		}
	} else {
		for(int i = 0; i < num_imus; i++){
			if(DEBUG) std::cout << "ENTERED FOR WRITE MEAS!!!!!!" << std::endl;
			norm_result += getNormFromMeas(meas_array.m[input_imu_array[i]]); 
		}
	}
	return norm_result;
}

/* Shifts the input vector and inserts the new measurement */
void shiftAndInsertMeas(){
	std::rotate(thumb_input.begin(), thumb_input.begin() + 1, thumb_input.end());
	thumb_input.back() = ld (thumb_meas);
	std::rotate(index_input.begin(), index_input.begin() + 1, index_input.end());
	index_input.back() = ld (index_meas);
	std::rotate(middle_input.begin(), middle_input.begin() + 1, middle_input.end());
	middle_input.back() = ld (middle_meas);
	std::rotate(ring_input.begin(), ring_input.begin() + 1, ring_input.end());
	ring_input.back() = ld (ring_meas);
	std::rotate(little_input.begin(), little_input.begin() + 1, little_input.end());
	little_input.back() = ld (little_meas);
	std::rotate(ref_input.begin(), ref_input.begin() + 1, ref_input.end());
	ref_input.back() = ld (ref_meas);
}

/* Choose correct finger considering the following:
	1) change finger_id to a finger only if there is a signal in it and its input is greater than all other fingers' */
void getCorrectCollision(){
	if(DEBUG) std::cout << "GOING TO CHOOSE THE CORRECT FINGER!" << std::endl;

	// Not Checking absolute threshold on input because it is not useful, instead the absolute
	// threshold will be used for the residuals later in the code

	// Checking for the imu with the biggest residual
	ld residual_vec[NUM_FINGERS] = {thumb_output["output_residuals"][NUM_LAG], 
		index_output["output_residuals"][NUM_LAG], middle_output["output_residuals"][NUM_LAG],
		ring_output["output_residuals"][NUM_LAG], little_output["output_residuals"][NUM_LAG]};

	ld biggest_residual = 0.0;

	for(int i = 0; i < NUM_FINGERS; i++){
		if(residual_vec[i] > biggest_residual){
			biggest_residual = residual_vec[i];
			biggest_finger_id = i + 1;
		}
	}

	// Check if residuals bigger than an abs threshold
	abs_thresh_1 = thumb_output["output_residuals"][NUM_LAG] > ABS_MAX_THRESH;
	abs_thresh_2 = index_output["output_residuals"][NUM_LAG] > ABS_MAX_THRESH;
	abs_thresh_3 = middle_output["output_residuals"][NUM_LAG] > ABS_MAX_THRESH;
	abs_thresh_4 = ring_output["output_residuals"][NUM_LAG] > ABS_MAX_THRESH;
	abs_thresh_5 = little_output["output_residuals"][NUM_LAG] > ABS_MAX_THRESH;

	// Check reference imu
	ref_thresh = (ref_meas >= REF_THRESH && ref_output["output_signals"][NUM_LAG] != 0);

	// Check if synergy position is beyond threshold
	syn_thresh_ok = (synergy_pos >= SYN_POS_THRESH && synergy_pos <= SYN_POS_THRESH_2);

	// Check collision conditions
	if(thumb_output["output_signals"][NUM_LAG] != 0 && biggest_finger_id == 1 && 
		abs_thresh_1 && thumb_meas > REF_THRESH && syn_thresh_ok){
		finger_id = 1;
		std::cout << "Collision found on finger " << finger_id << " (THUMB)!!!!" << std::endl;
	}else if(index_output["output_signals"][NUM_LAG] != 0 && biggest_finger_id == 2 && 
		abs_thresh_2 && index_meas > REF_THRESH && syn_thresh_ok){
		finger_id = 2;
		std::cout << "Collision found on finger " << finger_id << " (INDEX)!!!!" << std::endl;
	}else if(middle_output["output_signals"][NUM_LAG] != 0 && biggest_finger_id == 3 && 
		abs_thresh_3 && middle_meas > REF_THRESH && syn_thresh_ok){
		finger_id = 3;
		std::cout << "Collision found on finger " << finger_id << " (MIDDLE)!!!!" << std::endl;
	}else if(ring_output["output_signals"][NUM_LAG] != 0 && biggest_finger_id == 4 && 
		abs_thresh_4 && ring_meas > REF_THRESH && syn_thresh_ok){
		finger_id = 4;
		std::cout << "Collision found on finger " << finger_id << " (RING)!!!!" << std::endl;
	}else if(little_output["output_signals"][NUM_LAG] != 0 && biggest_finger_id == 5 && 
		abs_thresh_5 && little_meas > REF_THRESH && syn_thresh_ok){
		finger_id = 5;
		std::cout << "Collision found on finger " << finger_id << " (LITTLE)!!!!" << std::endl;
	}else {
		if(DEBUG) std::cout << "NO PEAKS ON ANY FINGER. WHAT SHOULD I DO?" << std::endl;
	}

	if(DEBUG) std::cout << "CORRECT FINGER WAS CHOSEN TO BE " << finger_id << "!!!" << std::endl;
}

/* If finger_id remains the same for too long, reset it to zero */
void resetFingerId(){
	if(DEBUG) std::cout << "ENTERED resetFingerId FUNCTION!!!!" << std::endl;

	// Start measuring time when finger_id changes
	if(old_finger_id != finger_id){
		finger_time = ros::Time::now();
		if(DEBUG) std::cout << "Starting to measure time because finger_id changed!!!!" << std::endl;
	}

	// If finger_id has been the same for more than timeout, reset it to zero
	ros::Duration timeout(FINGER_TIMEOUT);

	if(ros::Time::now() - finger_time > timeout){
		finger_id = 0;
		if(DEBUG) std::cout << "Resetting finger_id because timeout!!!!" << std::endl;
	}

	// Storing finger_id
	old_finger_id = finger_id;
}

/* Checks for other collision (checks all fingers) */
void checkForCollision(){
	if(DEBUG) std::cout << "THUMB INPUT IS " << thumb_input << std::endl;

	thumb_output = z_score_thresholding(thumb_input, int (NUM_LAG), ld (THRESHOLD), ld (INFLUENCE));
	index_output = z_score_thresholding(index_input, int (NUM_LAG), ld (THRESHOLD), ld (INFLUENCE));
	middle_output = z_score_thresholding(middle_input, int (NUM_LAG), ld (THRESHOLD), ld (INFLUENCE));
	ring_output = z_score_thresholding(ring_input, int (NUM_LAG), ld (THRESHOLD), ld (INFLUENCE));
	little_output = z_score_thresholding(little_input, int (NUM_LAG), ld (THRESHOLD), ld (INFLUENCE));
	ref_output = z_score_thresholding(ref_input, int (NUM_LAG), ld (THRESHOLD), ld (INFLUENCE));

	if(DEBUG) std::cout << "DONE Z SCORE FOR ALL FINGERS AND REFERENCE!" << std::endl;

	if(DEBUG) std::cout << "THUMB OUPUT IS " << thumb_output["output_signals"] << std::endl;

	// Checking collision according to current finger in collision
	getCorrectCollision();

	if(DEBUG) std::cout << "FINGER IDS SET CORRECTLY!" << std::endl;

	// If finger_id remains the same for too long, reset it to zero
	resetFingerId();
}

//-----------------------------------------------------------------------------------------------------------------------//
//CALLBACK FUNCTIONS
//-----------------------------------------------------------------------------------------------------------------------//
/* Callback for IMU topic subscriber */
void collisionDetection(const qb_interface::inertialSensorArrayConstPtr& input_sensor_array){
	// Getting size of imu arrays for passing them to writeMeasFromArray
	int size_array_ref = (sizeof(ref_imus)/sizeof(*ref_imus));
	int size_array_thumb = (sizeof(thumb_imus)/sizeof(*thumb_imus));
	int size_array_index = (sizeof(index_imus)/sizeof(*index_imus));
	int size_array_middle = (sizeof(middle_imus)/sizeof(*middle_imus));
	int size_array_ring = (sizeof(ring_imus)/sizeof(*ring_imus));
	int size_array_little = (sizeof(little_imus)/sizeof(*little_imus));

	// Save current measurements weighting them according to predefined weights and inverse of hand velocity
	ref_meas = float (REF_WEIGHT) * writeMeasFromArray(*input_sensor_array, ref_imus, size_array_ref);
	thumb_meas = float (THUMB_WEIGHT) * writeMeasFromArray(*input_sensor_array, thumb_imus, size_array_thumb);
	index_meas = float (INDEX_WEIGHT) * writeMeasFromArray(*input_sensor_array, index_imus, size_array_index);
	middle_meas = float (MIDDLE_WEIGHT) * writeMeasFromArray(*input_sensor_array, middle_imus, size_array_middle);
	ring_meas = float (RING_WEIGHT) * writeMeasFromArray(*input_sensor_array, ring_imus, size_array_ring);
	little_meas = float (LITTLE_WEIGHT) * writeMeasFromArray(*input_sensor_array, little_imus, size_array_little);
	

	// Push measurements to the input vectors and increment num_saved_meas if necessary
	shiftAndInsertMeas();

	if(num_saved_meas <= NUM_LAG) num_saved_meas++;

	// If NUM_LAG + 1 measurements are available, perform the z score peak identification
	if(num_saved_meas > NUM_LAG){
		if(DEBUG) std::cout << "STARTING TO CHECK FOR COLLISIONS!" << std::endl;
		checkForCollision();
	}

	if(DEBUG) std::cout << "GOING TO PUBLISH NOW!" << std::endl;

	// Publish the current finger in collision
	finger_id_msg.data = finger_id;
	pub_fing_id.publish(finger_id_msg);

	// Couting the finger in collision
	// std::cout << "The collision finger id is " << finger_id << "!!!!" << std::endl;
}

/* Callback for SYNERGY topic subscriber */
void getSyn(const sensor_msgs::JointState& curr_joint_states){
	// Get synergy position and set it
	synergy_pos = 
		curr_joint_states.position[find (curr_joint_states.name.begin(),curr_joint_states.name.end(), 
			std::string(HAND_JOINT)) - curr_joint_states.name.begin()];

	if(synergy_pos < 0.05 && first_below){
		std::cout << "------ HAND OPEN ------" << std::endl;
		first_below = false;
		first_above = true;
	} else if(synergy_pos > 0.95 && first_above){
		std::cout << "------ HAND CLOSED ------" << std::endl;
		first_above = false;
		first_below = true;
	}

	// Get synergy velocity and set it
	synergy_vel = 
		curr_joint_states.velocity[find (curr_joint_states.name.begin(),curr_joint_states.name.end(), 
			std::string(HAND_JOINT)) - curr_joint_states.name.begin()];

	// Saturation of velocity to avoid problems during inversion
	if(synergy_vel > SYN_VEL_SAT_MAX) synergy_vel = float (SYN_VEL_SAT_MAX);
  	else if(synergy_vel < SYN_VEL_SAT_MIN) synergy_vel = float (SYN_VEL_SAT_MIN);

	// Couting the synergy values
	if(DEBUG) std::cout << "The synergy_pos and synergy_vel are (" << synergy_pos << ", " << synergy_vel << ")." << std::endl;
}


//-----------------------------------------------------------------------------------------------------------------------//
// MAIN
//-----------------------------------------------------------------------------------------------------------------------//

int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "finger_collision_identification");
	ros::NodeHandle cd_nh;

	// Getting params from parameter server
	if(!getParamsOfYaml()) ROS_WARN("Some params not loaded correctly!");

	// Creating a ROS publisher for the output for publishing the finger_id of the finger in collision
	pub_fing_id = cd_nh.advertise<std_msgs::Int8>(std::string(OUTPUT_TOPIC), 10);

	// Creating a ROS subscriber for the input imu acceleration topic
	ros::Subscriber imu_sub = cd_nh.subscribe(std::string(INPUT_TOPIC), 10, collisionDetection);

	// Creating a ROS subscriber to get synergy_joint state
	ros::Subscriber syn_sub = cd_nh.subscribe(std::string(SYN_TOPIC), 10, getSyn);

	// Success message
	std::cout << "The collision finger id is being published!" << std::endl;

	// Spin
	ros::spin();

}