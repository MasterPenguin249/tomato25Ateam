#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>
#include "DynamixelControl/DynamixelControl.h"
#include <yolo_detection/BoundingBoxArray.h>
#include <yolo_detection/BoundingBox.h>
#include <geometry_msgs/PointStamped.h>

#include <cmath>
#include <memory>
#include <string>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdexcept>
#include <vector>
#include <deque>

#define NODE_FREQUENCY        200

#define f1(x, y) -(cos(x) / cos(y))
#define f2(x, y) (sin(x) / sin(y))
#define rad(x) (x * M_PI/180)

// Enhanced picking states
enum PickingPhase {
    PHASE_APPROACH,     // Moving towards tomato
    PHASE_FINE_ADJUST,  // Microadjustments near tomato
    PHASE_GRASP,        // Closing gripper
    PHASE_LIFT,         // Lifting with tomato
    PHASE_SHAKE_TEST,   // Gentle shake to test grip
    PHASE_RETURN,       // Moving back to start
    PHASE_IDLE          // Waiting for next command
};

// Enhanced pick detection parameters
struct PickDetection {
    double velocity_threshold = 0.005;      // Gripper velocity threshold (rad/s)
    double position_error_threshold = 0.1;  // Position error indicating resistance
    double force_threshold_min = 0.05;      // Minimum force indicating contact
    double force_threshold_max = 10.0;      // Maximum safe force
    int stability_count_required = 15;      // Frames of stability needed
    int contact_confirmation_frames = 8;    // Frames to confirm contact
    
    // Current state
    std::deque<double> recent_positions;
    std::deque<double> recent_targets;
    std::deque<double> recent_velocities;
    std::deque<double> recent_errors;
    int stability_counter = 0;
    int contact_counter = 0;
    int stall_counter = 0;
    bool tomato_detected_in_grip = false;
    bool pick_successful = false;
    double current_grip_velocity = 0.0;
    double current_grip_force = 0.0;
    double current_position_error = 0.0;
    double last_position = 0.0;
    
    void reset() {
        recent_positions.clear();
        recent_targets.clear();
        recent_velocities.clear();
        recent_errors.clear();
        stability_counter = 0;
        contact_counter = 0;
        stall_counter = 0;
        tomato_detected_in_grip = false;
        pick_successful = false;
        current_grip_velocity = 0.0;
        current_grip_force = 0.0;
        current_position_error = 0.0;
        last_position = 0.0;
    }
};


// Variable speed control
struct SpeedControl {
    double base_speed_x = 0.02;
    double base_speed_y = 0.02;
    double approach_speed_multiplier = 2.0;    // Faster when far
    double fine_adjust_speed_multiplier = 0.3; // Slower when close
    double micro_adjust_speed_multiplier = 0.1; // Very slow for final positioning
    
    double distance_threshold_approach = 8.0;   // cm
    double distance_threshold_fine = 3.0;       // cm
    double distance_threshold_micro = 1.5;      // cm
    
    double getCurrentSpeedMultiplier(double distance_to_target) {
        if (distance_to_target > distance_threshold_approach) {
            return approach_speed_multiplier;
        } else if (distance_to_target > distance_threshold_fine) {
            return 1.0; // Base speed
        } else if (distance_to_target > distance_threshold_micro) {
            return fine_adjust_speed_multiplier;
        } else {
            return micro_adjust_speed_multiplier;
        }
    }
};

// tomato states
double tomato_x = 9;
double tomato_y = 9;
double tomato_z = 0;
double tomato_size = 2.8; //cm
double t = 0;
bool use_realsense = true;  // Toggle between RealSense and monocular
geometry_msgs::PointStamped latest_tomato_point;
bool has_realsense_data = false;

// Enhanced picking control
PickingPhase current_phase = PHASE_IDLE;
PickDetection pick_detector;
SpeedControl speed_controller;
double phase_timer = 0.0;
bool enable_pick_detection = true;

// arm states
float vel_ax_x = 0.0;
float vel_ax_y = 0.0;
bool arrived = false;
bool paused = false;
bool backed = false;
bool fixed_y = true;
bool goback = false;
double closed = rad(-40);
double opened = rad(10);
double l = 9; // cm

// Tomato position tracking for microadjustments
struct TomatoTracker {
    std::deque<std::vector<double>> recent_positions;
    int max_history = 10;
    double position_variance_threshold = 0.5; // cm
    
    void addPosition(double x, double y, double z) {
        std::vector<double> pos = {x, y, z};
        recent_positions.push_back(pos);
        if (recent_positions.size() > max_history) {
            recent_positions.pop_front();
        }
    }
    
    std::vector<double> getPredictedPosition() {
        if (recent_positions.empty()) return {tomato_x, tomato_y, tomato_z};
        
        // Simple moving average with recent positions weighted more
        std::vector<double> weighted_pos = {0, 0, 0};
        double total_weight = 0;
        
        for (int i = 0; i < recent_positions.size(); i++) {
            double weight = (i + 1.0) / recent_positions.size(); // More recent = higher weight
            for (int j = 0; j < 3; j++) {
                weighted_pos[j] += recent_positions[i][j] * weight;
            }
            total_weight += weight;
        }
        
        for (int j = 0; j < 3; j++) {
            weighted_pos[j] /= total_weight;
        }
        
        return weighted_pos;
    }
    
    bool isPositionStable() {
        if (recent_positions.size() < 5) return false;
        
        // Calculate variance in recent positions
        std::vector<double> mean = {0, 0, 0};
        for (const auto& pos : recent_positions) {
            for (int i = 0; i < 3; i++) mean[i] += pos[i];
        }
        for (int i = 0; i < 3; i++) mean[i] /= recent_positions.size();
        
        double variance = 0;
        for (const auto& pos : recent_positions) {
            for (int i = 0; i < 3; i++) {
                variance += pow(pos[i] - mean[i], 2);
            }
        }
        variance /= (recent_positions.size() * 3);
        
        return sqrt(variance) < position_variance_threshold;
    }
};

TomatoTracker tomato_tracker;

//joy states
bool opening = true;
bool autonomous = false;
int prev_start_button = 0;
int prev_back_button = 0;
float lt = 0;

// camera states
double cam_angle = rad(0);
double armdim_x = 8;
double armdim_y = 9;

// velocity variables
int16_t vel_mx_write= 0; // -285 ~ 285
float scale_ax = 0.05;
float vel_ax_1 = 0.0;
float vel_ax_2 = 0.0;
double speed_x = 0.02;
double speed_y = 0.02;
float scale_mx = 30.0; // 3/s? that's 30/ds

// mx position (cm)
float mx_pos = 3.0;
double worm_pitch = 1.5;

// Fixed pick detection function
bool detectTomatoPick(const std::vector<double>& current_values, const std::vector<double>& target_values) {
    if (!enable_pick_detection || current_phase != PHASE_GRASP) return false;
    
    double current_position = current_values[2];
    double target_position = target_values[2];
    double position_error = abs(target_position - current_position);
    
    // Calculate gripper velocity
    pick_detector.current_grip_velocity = (current_position - pick_detector.last_position) / 0.005; // 200Hz
    pick_detector.last_position = current_position;
    pick_detector.current_position_error = position_error;
    
    // Store recent values
    pick_detector.recent_positions.push_back(current_position);
    pick_detector.recent_targets.push_back(target_position);
    pick_detector.recent_velocities.push_back(abs(pick_detector.current_grip_velocity));
    pick_detector.recent_errors.push_back(position_error);
    
    // Keep only recent data (last 1 second at 200Hz)
    int max_samples = 200;
    if (pick_detector.recent_positions.size() > max_samples) {
        pick_detector.recent_positions.pop_front();
        pick_detector.recent_targets.pop_front();
        pick_detector.recent_velocities.pop_front();
        pick_detector.recent_errors.pop_front();
    }
    
    // Need enough data to make decisions
    if (pick_detector.recent_positions.size() < 20) return false;
    
    // Detection Method 1: Velocity stall detection
    // When gripper hits tomato, velocity should drop significantly
    double avg_velocity = 0;
    for (int i = std::max(0, (int)pick_detector.recent_velocities.size() - 10); 
         i < pick_detector.recent_velocities.size(); i++) {
        avg_velocity += pick_detector.recent_velocities[i];
    }
    avg_velocity /= std::min(10, (int)pick_detector.recent_velocities.size());
    
    bool velocity_stalled = avg_velocity < pick_detector.velocity_threshold;
    
    // Detection Method 2: Position error increase
    // When gripper contacts tomato, it can't reach target as easily
    double avg_error = 0;
    for (int i = std::max(0, (int)pick_detector.recent_errors.size() - 10); 
         i < pick_detector.recent_errors.size(); i++) {
        avg_error += pick_detector.recent_errors[i];
    }
    avg_error /= std::min(10, (int)pick_detector.recent_errors.size());
    
    bool position_error_high = avg_error > pick_detector.position_error_threshold;
    
    // Detection Method 3: Target is still decreasing but position plateaued
    bool target_still_closing = false;
    bool position_plateaued = false;
    
    if (pick_detector.recent_targets.size() >= 10) {
        double target_change = pick_detector.recent_targets.back() - 
                              pick_detector.recent_targets[pick_detector.recent_targets.size() - 10];
        target_still_closing = target_change < -0.01; // Target decreasing (closing)
        
        double position_change = pick_detector.recent_positions.back() - 
                               pick_detector.recent_positions[pick_detector.recent_positions.size() - 10];
        position_plateaued = abs(position_change) < 0.005; // Position not changing much
    }
    
    // Combined detection logic
    bool contact_detected = (velocity_stalled && position_error_high) || 
                           (target_still_closing && position_plateaued && position_error_high);
    
    if (contact_detected) {
        pick_detector.contact_counter++;
        pick_detector.stall_counter++;
    } else {
        pick_detector.contact_counter = std::max(0, pick_detector.contact_counter - 1);
        pick_detector.stall_counter = 0;
    }
    
    // Confirm contact over multiple frames
    bool contact_confirmed = pick_detector.contact_counter >= pick_detector.contact_confirmation_frames;
    
    // Check for position stability after contact
    if (contact_confirmed) {
        double pos_variance = 0;
        if (pick_detector.recent_positions.size() >= 10) {
            double mean_pos = 0;
            for (int i = pick_detector.recent_positions.size() - 10; i < pick_detector.recent_positions.size(); i++) {
                mean_pos += pick_detector.recent_positions[i];
            }
            mean_pos /= 10;
            
            for (int i = pick_detector.recent_positions.size() - 10; i < pick_detector.recent_positions.size(); i++) {
                pos_variance += pow(pick_detector.recent_positions[i] - mean_pos, 2);
            }
            pos_variance /= 10;
        }
        
        if (sqrt(pos_variance) < 0.01) {
            pick_detector.stability_counter++;
        } else {
            pick_detector.stability_counter = 0;
        }
    }
    
    bool position_stable = pick_detector.stability_counter >= pick_detector.stability_count_required;
    
    // Final pick detection
    pick_detector.tomato_detected_in_grip = contact_confirmed && position_stable;
    
    // Additional check: maintain detection if we're lifting and still have resistance
    if (current_phase == PHASE_LIFT && pick_detector.tomato_detected_in_grip) {
        pick_detector.pick_successful = true;
    }
    
    return pick_detector.pick_successful;
}

// Modified gripper control for gradual closing
void gradualGripperControl(std::vector<double> &target_val, const std::vector<double> &current_val) {
    static double grip_start_time = 0;
    static bool grip_initialized = false;
    static double grip_start_position = opened;
    
    if (current_phase == PHASE_GRASP) {
        if (!grip_initialized) {
            grip_start_time = t;
            grip_start_position = current_val[2];
            grip_initialized = true;
        }
        
        double grip_duration = t - grip_start_time;
        double grip_speed = 0.02; // rad/s - adjust for desired closing speed
        
        // Gradual closing
        static double desired_position = grip_start_position;
        desired_position -= (grip_speed * grip_duration);
        
        // Don't close beyond safe limit
        desired_position = std::max(closed + 0.1, desired_position); // Leave some margin
        
        target_val[2] = desired_position;
        
        // If we detect a pick, stop closing further
        if (pick_detector.tomato_detected_in_grip) {
            // Hold current position
            target_val[2] = current_val[2];
        }
        
    } else {
        grip_initialized = false;
        // Normal gripper control for other phases
        target_val[2] = opening ? opened : closed;
    }
}


// Robust shake test to verify grip
bool performShakeTest(std::vector<double>& target_val, const std::vector<double>& current_val) {
    static double shake_start_time = 0;
    static bool shake_initialized = false;
    static double original_z = 0;
    
    if (!shake_initialized) {
        shake_start_time = t;
        original_z = target_val[4];
        shake_initialized = true;
    }
    
    double shake_duration = t - shake_start_time;
    double shake_amplitude = 0.1; // Small shake amplitude
    double shake_frequency = 2.0; // Hz
    
    // Perform gentle shake
    if (shake_duration < 2.0) { // Shake for 2 seconds
        target_val[4] = original_z + shake_amplitude * sin(2 * M_PI * shake_frequency * shake_duration);
        
        // Monitor for tomato loss during shake
        if (pick_detector.current_grip_force < pick_detector.force_threshold_min * 0.5) {
            // Lost grip during shake
            shake_initialized = false;
            return false;
        }
        return false; // Still shaking
    } else {
        // Shake complete
        target_val[4] = original_z;
        shake_initialized = false;
        
        // Check if tomato is still gripped
        bool still_gripped = pick_detector.current_grip_force > pick_detector.force_threshold_min;
        return still_gripped;
    }
}

// arrived (x, y)
bool is_arrived(std::vector<double> &cur_val, std::vector<double> &target_val){
  for(int i = 0; i <5; i++){
    if(i == 2 || i== 3) continue;
    if(abs(cur_val[i] - target_val[i]) < 0.08) continue;
    else return false;
  }
  return true;
}

// Enhanced arrival check with distance-based threshold
bool is_arrived_enhanced(const std::vector<double> &cur_val, const std::vector<double> &target_val, double distance_to_target) {
    double threshold = 0.08;
    
    // Tighter threshold when close to tomato
    if (current_phase == PHASE_FINE_ADJUST || current_phase == PHASE_GRASP) {
        threshold = 0.03;
    }
    
    for(int i = 0; i < 5; i++){ // Check only first 2 joints
        if(i == 2 || i== 3) continue;
        if(abs(cur_val[i] - target_val[i]) > threshold) {
            return false;
        }
    }
    return true;
}

// reset arm position
void go_back(std::vector<double> &cur_val, std::vector<double> &target_val){
  target_val[0] = rad(-75);
  target_val[1] = rad(105);
  target_val[4] = rad(75);
  std::vector<double> init_state = {rad(-75), rad(105), opened, 0, rad(75)};
  backed = is_arrived(cur_val, init_state);
  if(backed){
    ROS_INFO("Returned to home position - ready for next pick");
    arrived = false;
    paused = false;
    opening = true;
    goback = false;
    current_phase = PHASE_IDLE;
    pick_detector.reset();
    t = 0;
    phase_timer = 0;
  } 
}

void joyCallback(const sensor_msgs::Joy& msg)
{
  // cross 
  if(msg.buttons[1]==1)
    vel_ax_1 = - msg.buttons[1]*scale_ax;
  else if(msg.buttons[2]==1)
    vel_ax_1 = msg.buttons[2]*scale_ax;
  else
    vel_ax_1 = 0;

  // arm x, y
  vel_ax_x = msg.axes[4];
  vel_ax_y = msg.axes[7];
  opening = msg.buttons[5] == 0;

  // mx
  if(msg.buttons[3]==1)
    vel_mx_write= msg.buttons[3]*scale_mx;    
  else if(msg.buttons[0]==1)
    vel_mx_write= -msg.buttons[0]*scale_mx;
  else
    vel_mx_write=0;

  // fixed y or not
  fixed_y = (msg.axes[2]< -0.8)?  false: true;

  int current_start_button = msg.buttons[7];
  // Detect button pressed (start)
  if (current_start_button == 1 && prev_start_button == 0) {
    autonomous = !autonomous;
    if (autonomous) {
        current_phase = PHASE_APPROACH;
        pick_detector.reset();
        phase_timer = 0;
        ROS_INFO("Starting autonomous tomato picking");
    } else {
        current_phase = PHASE_IDLE;
        ROS_INFO("Stopping autonomous mode");
    }
  }

  int current_back_button = msg.buttons[6];
  // Detect button pressed (back)
  if (current_back_button == 1 && prev_back_button == 0) goback = true;
  
  // Update previous button state
  prev_start_button = current_start_button;
  prev_back_button = current_back_button;
}

void realsensePointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    if(paused && current_phase != PHASE_FINE_ADJUST) return;
    
    latest_tomato_point = *msg;
    has_realsense_data = true;
    
    tomato_x = msg->point.x * 100.0;
    tomato_y = msg->point.y * -100.0;
    tomato_z = msg->point.z * -100.0;
    
    // Add to tracker for microadjustments
    tomato_tracker.addPosition(tomato_x, tomato_y, tomato_z);
}

void bboxCallback(const yolo_detection::BoundingBoxArray::ConstPtr& msg)
{
  if(paused && current_phase != PHASE_FINE_ADJUST) return;
  double min_distance = 10000;

  if(use_realsense && has_realsense_data) {
    return;  // RealSense data is handled in realsensePointCallback
  }

  // Process each bounding box
  for (size_t i = 0; i < msg->bounding_boxes.size(); ++i)
  {
    const yolo_detection::BoundingBox& bbox = msg->bounding_boxes[i];
    double distance = tomato_size /((bbox.x_max - bbox.x_min) + (bbox.y_max - bbox.y_min))/2.0 *3111;
    if(bbox.class_name != "tomato" || distance > min_distance){
      continue;
    }else if(abs(bbox.x_max - bbox.x_min - (bbox.y_max-bbox.y_min)) > 0.1*std::max(bbox.x_max - bbox.x_min, bbox.y_max-bbox.y_min)){
      continue;
    }
    double center_x = (bbox.x_max + bbox.x_min)/2.0;
    double center_y = (bbox.y_max + bbox.y_min)/2.0;
    //tomato_x within (7, 21)
    tomato_x = distance * cos(cam_angle) - (240 - center_y )/480 * 11.25 / 17 * distance * sin(cam_angle)- 3; 
    // tomato_y has to be within (0, 14)
    tomato_y = (240 - center_y )/480 * 11.25 / 17 * distance * cos(cam_angle)\
    + distance * sin(cam_angle) + (41.5 - mx_pos); // calibration needed
    //tomato_z within (-5.5, 5.5)
    tomato_z = (320 - center_x )/640 * 15 / 17 * distance + 1;
    
    // Add to tracker
    tomato_tracker.addPosition(tomato_x, tomato_y, tomato_z);
  }
}

std::vector<std::pair<double, double>> square{
  {5, 5}, {5, 9}, {9, 9}, {9, 5} 
};
std::vector<std::pair<double, double>> farstraight{
  {9, -10}, {9, 10}
};
int point = 0;

void limitcheck(std::vector<double> &target_val){
  // upper and lower rotational limits
  std::vector<std::pair<double, double>> limits = {
    {rad(-70), rad(80)},
    {rad(-target_val[0]+10), rad(-target_val[0]+140)},
    {rad(-150), rad(150)},
    {-100, 100},
    {rad(20), rad(120)}
  };

  // gatekeep
  for(int i = 0; i < target_val.size(); i++){
    if(target_val[i] < limits[i].first){
       target_val[i] = limits[i].first;
    }else if (target_val[i] > limits[i].second){
      target_val[i] = limits[i].second;
    }
  }
}

bool is_ok(double theta1, double theta2){
  double _x = l*(sin(theta1) + sin(theta2));
  double _y = l*(cos(theta1) - cos(theta2));
  if(sqrt(_x*_x+_y*_y) > 2*l || _x > 16 || _y < -10) {
    ROS_ERROR("invalid square values");
    return false;
  }
  return true;
}

// Enhanced go_to with variable speed
void go_to_enhanced(double _x, double _y, double _z, std::vector<double> &target_val, 
                   const std::vector<double> &current_val, double speed_multiplier = 1.0){
  std::vector<double> goal(3);
  if(!is_ok(_x, _y)) return;
  
  //convert to angle
  goal[1] = asin(sqrt((_x*_x+_y*_y))/2/l) + atan(_y/_x);
  goal[0] = asin(sqrt((_x*_x+_y*_y))/2/l) - atan(_y/_x);
  goal[2] = _z/2.5;
  
  // Apply speed control for smooth approach
  double speed_x_adj = speed_x * speed_multiplier;
  double speed_y_adj = speed_y * speed_multiplier;
  
  // Gradual movement towards target
  if(abs(target_val[0] - goal[0]) > speed_x_adj) {
    target_val[0] += (goal[0] > target_val[0]) ? speed_x_adj : -speed_x_adj;
  } else {
    target_val[0] = goal[0];
  }
  
  if(abs(target_val[1] - goal[1]) > speed_y_adj) {
    target_val[1] += (goal[1] > target_val[1]) ? speed_y_adj : -speed_y_adj;
  } else {
    target_val[1] = goal[1];
  }
  
  if(abs(target_val[4] - goal[2]) > scale_ax) {
    target_val[4] += (goal[2] > target_val[4]) ? scale_ax : -scale_ax;
  } else {
    target_val[4] = goal[2];
  }
}

// insert coordinates, arm goes there
void go_to(double _x, double _y, double _z, std::vector<double> &target_val){
  std::vector<double> goal(3);
  if(!is_ok(_x, _y)) return;
  //convert to angle (I got them flipped the entire time)
  goal[1] = asin(sqrt((_x*_x+_y*_y))/2/l) + atan(_y/_x);
  goal[0] = asin(sqrt((_x*_x+_y*_y))/2/l) - atan(_y/_x);
  goal[2] = _z/2.5;
  target_val[0] = goal[0];
  target_val[1] = goal[1];
  target_val[4] = goal[2];
}

void make_move(std::vector<double> &target_val, std::vector<double> &theta){
  double signx = vel_ax_x >= 0? 1:-1;
  double signy = vel_ax_y >= 0? 1:-1;
  double x = l*(sin(theta[0]) + sin(theta[1]));
  double y = l*(cos(theta[0]) - cos(theta[1]));
  double z = -theta[4] * 2.5;
  
  double dtheta1 = signx * speed_x;
  double dtheta2 = signy * speed_y;

  // Manual joy stick control (overrides autonomous when active)
  if(!autonomous && (abs(vel_ax_x) > 0.8 || abs(vel_ax_y) > 0.8)) {
    if(abs(vel_ax_x) > 0.8 && is_ok(target_val[0] + dtheta1, target_val[1] + dtheta1 * sin(theta[0])/sin(theta[1]))){
      target_val[0] += dtheta1;
      target_val[1] += dtheta1 * sin(theta[0])/sin(theta[1]);
      t = 0;
    }
    else if(abs(vel_ax_y) > 0.8 && !fixed_y && is_ok(target_val[0] - dtheta2 * cos(theta[1])/cos(theta[0]), target_val[1] + dtheta2)){ 
      target_val[1] += dtheta2;
      target_val[0] += -dtheta2 * cos(theta[1])/cos(theta[0]);
      t = 0;
    }
  }
  // Enhanced autonomous picking with phases
  else if(autonomous && current_phase != PHASE_IDLE){
    
    // Calculate distance to target
    double target_x = tomato_x - armdim_x;
    double target_y = tomato_y;
    double target_z = tomato_z;
    
    // Use predicted position for fine adjustments
    if (current_phase == PHASE_FINE_ADJUST) {
        auto predicted_pos = tomato_tracker.getPredictedPosition();
        target_x = predicted_pos[0] - armdim_x;
        target_y = predicted_pos[1];
        target_z = predicted_pos[2];
    }
    
    double distance_to_target = sqrt(pow(x - target_x, 2) + pow(y - target_y, 2));
    
    switch(current_phase) {
        case PHASE_APPROACH:
            {
                double speed_mult = speed_controller.getCurrentSpeedMultiplier(distance_to_target);
                go_to_enhanced(target_x, target_y, target_z, target_val, theta, speed_mult);
                
                // Transition to fine adjustment phase when close
                if (distance_to_target < speed_controller.distance_threshold_fine) {
                    current_phase = PHASE_FINE_ADJUST;
                    phase_timer = 0;
                    ROS_INFO("Entering fine adjustment phase");
                }
                
                // Safety timeout
                if (phase_timer > 10.0) {
                    ROS_WARN("Approach phase timeout, switching to fine adjust");
                    current_phase = PHASE_FINE_ADJUST;
                    phase_timer = 0;
                }
            }
            break;
            
        case PHASE_FINE_ADJUST:
            {
                // Very slow, precise movements with tomato tracking
                double speed_mult = speed_controller.micro_adjust_speed_multiplier;
                go_to_enhanced(target_x, target_y, target_z, target_val, theta, speed_mult);
                
                bool position_stable = tomato_tracker.isPositionStable();
                bool close_enough = distance_to_target < speed_controller.distance_threshold_micro;
                
                if (close_enough && position_stable && phase_timer > 1.0) {
                    current_phase = PHASE_GRASP;
                    phase_timer = 0;
                    opening = false; // Start closing gripper
                    ROS_INFO("Starting grasp phase");
                }
                
                // Timeout for fine adjustment
                if (phase_timer > 8.0) {
                    ROS_WARN("Fine adjustment timeout, proceeding to grasp");
                    current_phase = PHASE_GRASP;
                    phase_timer = 0;
                    opening = false;
                }
            }
            break;
            
        case PHASE_GRASP:
            {
                // Hold position while closing gripper
                opening = false;
                
                // Check for successful pick
                bool pick_detected = detectTomatoPick(theta, target_val);
                
                if (pick_detected || phase_timer > 3.0) {
                    if (pick_detected) {
                        ROS_INFO("Tomato successfully grasped!");
                        current_phase = PHASE_LIFT;
                    } else {
                        ROS_WARN("Grasp timeout, proceeding to lift phase");
                        current_phase = PHASE_LIFT;
                    }
                    phase_timer = 0;
                }
            }
            break;
            
        case PHASE_LIFT:
            {
                // Lift tomato slowly
                target_val[4] += 0.01; // Slow upward movement
                
                // Monitor grip during lift
                bool still_gripped = detectTomatoPick(theta, target_val);
                
                if (phase_timer > 2.0) {
                    if (still_gripped) {
                        current_phase = PHASE_SHAKE_TEST;
                        ROS_INFO("Starting shake test");
                    } else {
                        ROS_WARN("Lost grip during lift, returning to start");
                        current_phase = PHASE_RETURN;
                    }
                    phase_timer = 0;
                }
            }
            break;
            
        case PHASE_SHAKE_TEST:
            {
                bool shake_passed = performShakeTest(target_val, theta);
                
                if (phase_timer > 3.0) { // Shake test complete
                    if (shake_passed) {
                        ROS_INFO("Pick successful! Tomato securely held");
                        pick_detector.pick_successful = true;
                        current_phase = PHASE_RETURN;
                    } else {
                        ROS_WARN("Shake test failed, lost tomato");
                        current_phase = PHASE_RETURN;
                    }
                    phase_timer = 0;
                }
            }
            break;
            
        case PHASE_RETURN:
        case PHASE_IDLE:
            // Handle return in the existing logic below
            break;
    }
    
    arrived = is_arrived_enhanced(theta, target_val, distance_to_target);
  }

  // Return to home logic
  if((current_phase == PHASE_RETURN) || goback){
    go_back(theta, target_val);
  }

  // Gripper control
  if (current_phase == PHASE_GRASP) {
        gradualGripperControl(target_val, theta);
    } else {
        target_val[2] = opening ? opened : closed;
    }

  // Base rotation and other controls
  mx_pos = theta[3] * worm_pitch;
  target_val[3] += rad(0.02);
  target_val[4] += vel_ax_1; 
  
  // Logging and status updates
  if(abs(t - round(t)) < 0.03){
    ROS_INFO("Phase: %d, tomato_xyz: [%f, %f, %f]", current_phase, tomato_x, tomato_y, tomato_z);
    ROS_INFO("ARM_COORDS: [%f, %f, %f], Distance: %f", x, y, z, 
             sqrt(pow(x - (tomato_x - armdim_x), 2) + pow(y - tomato_y, 2)));
    ROS_INFO("Grip Force: %f, Pick Success: %s", 
             pick_detector.current_grip_force, 
             pick_detector.pick_successful ? "YES" : "NO");
    
    if(tomato_x > 17) ROS_ERROR("TOO FAR !! move closer");
    
    // Phase-specific logging
    switch(current_phase) {
        case PHASE_APPROACH:
            ROS_INFO("APPROACHING - Speed mult: %f", 
                     speed_controller.getCurrentSpeedMultiplier(
                         sqrt(pow(x - (tomato_x - armdim_x), 2) + pow(y - tomato_y, 2))));
            break;
        case PHASE_FINE_ADJUST:
            ROS_INFO("FINE ADJUSTING - Stable: %s", 
                     tomato_tracker.isPositionStable() ? "YES" : "NO");
            break;
        case PHASE_GRASP:
            ROS_INFO("GRASPING - Contact: %d/%d", 
                     pick_detector.contact_counter, pick_detector.contact_confirmation_frames);
            break;
        case PHASE_LIFT:
            ROS_INFO("LIFTING - Maintaining grip");
            break;
        case PHASE_SHAKE_TEST:
            ROS_INFO("SHAKE TESTING - Verifying secure grip");
            break;
        case PHASE_RETURN:
            ROS_INFO("RETURNING to home position");
            break;
        default:
            break;
    }
  }

  // Update timers
  t += 0.005; // 200Hz update rate
  phase_timer += 0.005;

  limitcheck(target_val);
}

// Additional utility functions for enhanced picking

// Adaptive grip force control
void adaptiveGripControl(std::vector<double> &target_val, const std::vector<double> &current_val) {
    static double grip_integral = 0;
    static double last_grip_error = 0;
    
    if (current_phase == PHASE_GRASP || current_phase == PHASE_LIFT) {
        double desired_grip = closed + (opened - closed) * 0.3; // Partial close for tomato
        double current_grip = current_val[2];
        double grip_error = desired_grip - current_grip;
        
        // Simple PID control for grip
        double kp = 0.5, ki = 0.1, kd = 0.05;
        grip_integral += grip_error * 0.005;
        double grip_derivative = (grip_error - last_grip_error) / 0.005;
        
        double grip_adjustment = kp * grip_error + ki * grip_integral + kd * grip_derivative;
        
        // Apply limits to prevent damage
        grip_adjustment = std::max(-0.1, std::min(0.1, grip_adjustment));
        target_val[2] += grip_adjustment;
        
        last_grip_error = grip_error;
        
        // Clamp to safe range
        target_val[2] = std::max(closed, std::min(opened, target_val[2]));
    } else {
        grip_integral = 0;
        last_grip_error = 0;
    }
}

// Collision avoidance during approach
bool checkCollisionPath(double start_x, double start_y, double end_x, double end_y) {
    // Simple obstacle checking - extend as needed for your environment
    // This is a placeholder for more sophisticated collision detection
    
    // Check if path goes through known obstacles
    double path_length = sqrt(pow(end_x - start_x, 2) + pow(end_y - start_y, 2));
    int num_checks = std::max(5, (int)(path_length / 0.5)); // Check every 0.5cm
    
    for (int i = 0; i <= num_checks; i++) {
        double t_interp = (double)i / num_checks;
        double check_x = start_x + t_interp * (end_x - start_x);
        double check_y = start_y + t_interp * (end_y - start_y);
        
        // Add your specific collision checks here
        // For example, check against known obstacles in your workspace
        
        // Basic workspace bounds check
        if (check_x < 7 || check_x > 21 || check_y < 0 || check_y > 14) {
            return false; // Collision with workspace bounds
        }
    }
    
    return true; // Path is clear
}

// Emergency stop function
void emergencyStop(std::vector<double> &target_val, const std::vector<double> &current_val) {
    ROS_WARN("EMERGENCY STOP ACTIVATED");
    
    // Stop all movement
    for (int i = 0; i < target_val.size(); i++) {
        target_val[i] = current_val[i];
    }
    
    // Open gripper for safety
    target_val[2] = opened;
    
    // Reset state
    current_phase = PHASE_IDLE;
    autonomous = false;
    paused = false;
    pick_detector.reset();
}

// Force/torque monitoring for safety
bool monitorSafety(const std::vector<double> &current_val, const std::vector<double> &target_val) {
    // Monitor for excessive forces that might indicate collision or jam
    
    // Check for sudden position deviations (indicating collision)
    static std::vector<double> last_positions(5, 0);
    bool sudden_stop_detected = false;
    
    for (int i = 0; i < 2; i++) { // Check main arm joints
        double position_change = abs(current_val[i] - last_positions[i]);
        if (position_change > 0.2) { // Sudden large change threshold
            sudden_stop_detected = true;
            break;
        }
    }
    
    // Check grip force limits
    bool excessive_grip_force = pick_detector.current_grip_force > pick_detector.force_threshold_max;
    
    // Update last positions
    for (int i = 0; i < last_positions.size() && i < current_val.size(); i++) {
        last_positions[i] = current_val[i];
    }
    
    if (sudden_stop_detected) {
        ROS_WARN("Sudden position change detected - possible collision");
        return false;
    }
    
    if (excessive_grip_force) {
        ROS_WARN("Excessive grip force detected");
        return false;
    }
    
    return true; // All safety checks passed
}

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "enhanced_tomato_picker");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Rate cycle_rate(200); // 200Hz for smooth control
  
  ros::Subscriber subscriber = nh.subscribe("joy", 1, joyCallback);
  ros::Subscriber sub = nh.subscribe<yolo_detection::BoundingBoxArray>("tomato_detections", 1, bboxCallback);
  ros::Subscriber realsense_sub = nh.subscribe<geometry_msgs::PointStamped>("/realsense/tomato_3d_point", 1, realsensePointCallback);

  // Parameters
  pnh.param<bool>("use_realsense", use_realsense, true);
  pnh.param<bool>("enable_pick_detection", enable_pick_detection, true);
  pnh.param<double>("pick_force_threshold_min", pick_detector.force_threshold_min, 0.05);
  pnh.param<double>("pick_force_threshold_max", pick_detector.force_threshold_max, 2.0);
  pnh.param<double>("approach_speed_multiplier", speed_controller.approach_speed_multiplier, 2.0);
  pnh.param<double>("fine_adjust_speed_multiplier", speed_controller.fine_adjust_speed_multiplier, 0.3);

  std::string dev_name;
  pnh.param<std::string>("dev", dev_name, "/dev/ttyUSB0");
  DynamixelControl dynamixelcontrol(dev_name);

  // add motors
  dynamixelcontrol.addMotor("AX", 5);
  dynamixelcontrol.addMotor("AX", 4);
  dynamixelcontrol.addMotor("AX", 3);
  dynamixelcontrol.addMotor("MX", 10);
  dynamixelcontrol.addMotor("AX", 1);   //stage R/L
  
  dynamixelcontrol.torque_on();

  std::vector<double> target_positions{rad(-75), rad(105), opened, 0, rad(75)};
  std::vector<double> current_values(5);

  ROS_INFO("Enhanced Tomato Picker initialized. Press START to begin autonomous picking.");
  ROS_INFO("Features enabled: Variable speed control, Pick detection, Microadjustments, Robust picking");

  while(ros::ok())
  {
    ros::spinOnce();

    // Safety monitoring
    if (!monitorSafety(current_values, target_positions)) {
        emergencyStop(target_positions, current_values);
        continue;
    }

    // Enhanced movement control
    make_move(target_positions, current_values);
    
    // Adaptive grip control
    adaptiveGripControl(target_positions, current_values);
    
    // Apply control commands
    dynamixelcontrol.setTarget(target_positions); 
    dynamixelcontrol.write();
    dynamixelcontrol.read();  
    dynamixelcontrol.getCurrentvalues(current_values);
    
    cycle_rate.sleep();
  }
  
  dynamixelcontrol.torque_off();
  ROS_INFO("Enhanced Tomato Picker shutdown complete.");

  return 0;
}