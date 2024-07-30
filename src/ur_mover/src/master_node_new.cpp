
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>  /*controlling robot motion*/
#include <moveit_msgs/msg/collision_object.hpp>  /*For collision object messages*/
#include <moveit/planning_scene/planning_scene.h> /*For the planning scene*/
#include <moveit/planning_scene_interface/planning_scene_interface.h>  /*For interfacing with the planning scene*/
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "ur_custom_interfaces/msg/ur_command.hpp"  /*For custom messages defined in the ur_custom_interfaces package*/
using std::placeholders::_1;

using namespace std::chrono_literals;

using moveit::planning_interface::MoveGroupInterface;
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


class RobotMasterController : public rclcpp::Node
{
/*The constructor for the RobotMasterController class, which initializes the node and sets up initial states*/
  public:
    RobotMasterController(std::shared_ptr<rclcpp::Node> move_group_node, geometry_msgs::msg::Pose* lookout_pos, geometry_msgs::msg::Pose* item_drop_pos)  
    : Node("master_node"), is_lookout_position(false), is_horizontally_centered(false), 
    is_vertically_centered(false), is_moving(false), lookout_pos(lookout_pos), target_pose(*lookout_pos), prev_x(0),
    is_depth_reached(false), was_centered_message_shown(false), depth(0.0), item_drop_pose(item_drop_pos), is_at_item_position(false), 
    is_item_grabbed(false), is_item_picked(false), is_with_item_at_lookout_position(false)
    /*Initializing the pointers of the construct*/
    {
      RCLCPP_INFO(this->get_logger(), "Node started. Awaiting commands...");
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      RCLCPP_INFO(this->get_logger(), "SETUP LOGS");
      RCLCPP_INFO(this->get_logger(), "=======================================================");

      /*Creates a subscription to the custom_camera topic, with a callback to topic_callback.*/
      subscription_ = this->create_subscription<ur_custom_interfaces::msg::URCommand>(
      "custom_camera", 1, std::bind(&RobotMasterController::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Subscribed to /custom_camera topic");

      /*Creates a publisher for the custom_gripper topic - gripper control*/
      publisher = this->create_publisher<std_msgs::msg::String>("/custom_gripper", 1);
      RCLCPP_INFO(this->get_logger(), "Publisher on custom_gripper created");

      /*Initializes the MoveGroupInterface for controlling the robot arm*/
      move_group_ = new moveit::planning_interface::MoveGroupInterface(move_group_node, "ur_manipulator");
      auto const robot_pos = move_group_->getCurrentPose("wrist_3_link");
      RCLCPP_INFO(this->get_logger(), "Robot position: %f, %f, %f", robot_pos.pose.position.x, robot_pos.pose.position.y, robot_pos.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Robot rotation: %f, %f, %f, %f", 
      robot_pos.pose.orientation.x, robot_pos.pose.orientation.y, robot_pos.pose.orientation.z, robot_pos.pose.orientation.w
      );

      RCLCPP_INFO(this->get_logger(), "Lookout position: %f, %f, %f", lookout_pos->position.x, lookout_pos->position.y, lookout_pos->position.z);
      this->move_to_lookout_position();
      /*Keep in mind to check what is position.x,  position.y ... and orientation ....'*/
    }

/*The topic_callback function processes incoming messages from the custom_camera topic*/
  private:
    void topic_callback(const ur_custom_interfaces::msg::URCommand::SharedPtr msg)  /*take x,y,depth from  ur_custom interface and convert to int*/
    {
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      int x = std::stoi(msg->x);
      int y = std::stoi(msg->y);
      if(!is_moving) {
        depth = sanitize_depth(msg->depth);   /*The depth field is processed using a custom sanitize_depth function if the robot is not moving. 
        The received command values are then logged.*/
      }
      RCLCPP_INFO(this->get_logger(), "Received commands: x:%i, y: %i, depth: %f", x, y, depth);

      /*If the depth value is within the range of 0.0 to 0.5, it is added to the depths vector. We updated the value 0.5 to 0.7*/
      if(depth > 0.0 && depth < 0.7){
        depths.push_back(depth);
      }
      
      /*If the depth is reached and the robot is centered both horizontally and vertically, a message is logged indicating the robot is at
      the item position, and the function returns.*/
      if(is_depth_reached && is_horizontally_centered && is_vertically_centered) {
        RCLCPP_INFO(this->get_logger(), "At item position");
        return;
      }

      /*If the robot is already moving or is not in the lookout position, a message is logged and the function returns*/
      if(is_moving || !is_lookout_position){
        RCLCPP_INFO(this->get_logger(), "Robot is already moving. Ignoring command.");
        return;
      }

      /*If x is 0 and the robot is not already centered horizontally, a message is logged indicating the robot is already 
      centered horizontally. The is_horizontally_centered flag is set to true, the is_moving flag is set to false, and
       the robot's movement is stopped*/
      if(x == 0 && !is_horizontally_centered){
        RCLCPP_INFO(this->get_logger(), "Robot is already centered horizontally. Ignoring command.");
        is_horizontally_centered = true;
        is_moving = false;
        move_group_->stop();
        return;
      }

      /*If x is 1 and the robot is not moving and is not centered horizontally, the robot's target position in the 
      x-direction is incremented by 0.01, and a movement command is issued to move the robot to the right. If x is -1, 
      the target position in the x-direction is decremented by 0.01, and a movement command is issued to move the robot 
      to the left.*/
      if(x == 1 && !is_moving && !is_horizontally_centered){
        target_pose.position.x += 0.01;
        this->move(target_pose, "Moving robot to the right");

      } else if(x == -1 && !is_moving && !is_horizontally_centered){
        target_pose.position.x -= 0.01;
        this->move(target_pose, "Moving robot to the left");
      }

      // MOVING IN Y
      /*If the robot is not centered horizontally, a message is logged and the function returns*/
      if(!is_horizontally_centered){
        RCLCPP_INFO(this->get_logger(), "Robot is not centered horizontally. Ignoring command.");
        return;
      }

      if(y == 0 && !is_vertically_centered){
        RCLCPP_INFO(this->get_logger(), "Robot is already centered vertically. Ignoring command.");
        is_vertically_centered = true;
        move_group_->stop();
        is_moving = false;
        return;
      }

      if(y == 1 && !is_moving && !is_vertically_centered){
        target_pose.position.z -= 0.01;
        this->move(target_pose, "Moving robot to the bottom");
      } else if(y == -1 && !is_moving && !is_vertically_centered){
        target_pose.position.z += 0.01;
        this->move(target_pose, "Moving robot to the top");
      }

      /*This block checks if the robot is centered both horizontally and vertically. If it is and a message 
      indicating this has not yet been shown, a message is logged, was_centered_message_shown is set to true, 
      and a 3-second timer is started. If the robot is not centered, the function returns. The current time 
      is then updated.*/
      bool const is_robot_centered = is_horizontally_centered && is_vertically_centered;
      if(is_robot_centered && !was_centered_message_shown) {
        RCLCPP_INFO(this->get_logger(), "Robot is centered. Started timer.");
        was_centered_message_shown = true;
        end_timer = this->get_clock()->now() + rclcpp::Duration(3s);
      }
      else if(!is_robot_centered) {
        return;
      }
      timer = this->get_clock()->now();

      /*If the current time is less than the end time of the timer, the function returns*/
      if(timer < end_timer) {
        return;
      }

      /*If the depth is less than 0.01 or greater than 0.8, a message is logged indicating that the depth 
      reading is out of range, and the function returns.*/
      if(depth < 0.01) {
        RCLCPP_INFO(this->get_logger(), "Depth too small. Awaiting another reading.");
        return;
      }
      if (depth > 0.8) {
        RCLCPP_INFO(this->get_logger(), "Depth too big. Awaiting another reading.");
        return;
      }

      /*If the robot is not moving and the depth has not been reached, proceed to the next block of code*/
      // Reaching the item
      if(!is_moving && !is_depth_reached) {

        /*Apply a camera offset by adjusting the robot's target position in the z-direction. If the 
        movement is successful, a message is logged. Otherwise, an error message is logged and the node is shut down*/
        // including camera offset
        target_pose.position.z += 0.06;
        bool const offset_res = this->move(target_pose, "Applying camera offset");
        if(offset_res){
          RCLCPP_INFO(this->get_logger(), "Applied camera offset");
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not apply camera offset. Shutting down.");
          rclcpp::shutdown();
        }

        /*Move the robot forward by adjusting the target position in the y-direction using the depth value, camera 
        offset, and gripper offset. If the movement is successful, set is_depth_reached to true and log a success 
        message. Otherwise, log an error message and shut down the node.*/
        RCLCPP_INFO(this->get_logger(), "Moving robot forward by %f", depth);
        float camera_offset = 0.03;
        float gripper_offset = 0.00;
        target_pose.position.y += depth - camera_offset - gripper_offset;
        // shouldn't be hardcoded - offset in x when reaching item. we might not need this value.
        target_pose.position.x -= 0.03;

        bool const forward_res = this->move(target_pose, "Moving robot forward");

        if(forward_res){
          is_depth_reached = true;
          RCLCPP_INFO(this->get_logger(), "Arrived at item position.");
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not arrive at item position. Shutting down.");
          rclcpp::shutdown();
        }
      }

      /*If the depth is reached and the apple is not yet grabbed, log a message, wait for 1 second, 
      publish a message to close the gripper, wait for 5 seconds, log a message indicating the 
      gripper is closed, and set is_item_grabbed to true.*/
      // Grabbing the item
      if(is_depth_reached && !is_item_grabbed){
        RCLCPP_INFO(this->get_logger(), "About to close gripper");
        rclcpp::sleep_for(1s);
        publisher->publish(std_msgs::msg::String().set__data("close"));
        rclcpp::sleep_for(5s);
        RCLCPP_INFO(this->get_logger(), "Gripper closed");
        is_item_grabbed = true;
      }

      /*If the item is grabbed and the robot is not moving, adjust the target position to pick the item. 
      If the movement is successful, set is_item_picked to true and log a success message. 
      Otherwise, log an error message and shut down the node. target_pose.position.z += 0.03;: This increases
      the z coordinate by 0.03 units, moving the end effector slightly upward.target_pose.position.y -= 0.07;:
      This decreases the y coordinate by 0.07 units, moving the end effector slightly backward.*/
      // Picking the item
      if(is_item_grabbed && !is_moving){
        target_pose.position.z += 0.03;
        target_pose.position.y -= 0.07;
        bool const backward_res = this->move(target_pose, "Picking the item");
        if(backward_res){
          is_item_picked = true;
          RCLCPP_INFO(this->get_logger(), "Position after picking an item");
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not pick an item. Shutting down.");
          rclcpp::shutdown();
        }
      }

      /*Log a message before moving to the lookout position with the item. If the item is picked and 
      the robot is not moving, move to the lookout position, log a success message, and set 
      is_with_item_at_lookout_position to true. Log another message after moving to the lookout position.*/
      
          RCLCPP_INFO(this->get_logger(), "Before going to lookout position with item");
      // Going back to lookout position with item
      if(is_item_picked && !is_moving){
        this->move_to_lookout_position();
          RCLCPP_INFO(this->get_logger(), "Going to lookout position");
        is_with_item_at_lookout_position = true;
      }
          RCLCPP_INFO(this->get_logger(), "After going to lookout position with item");

      /*If the robot is at the lookout position with the item and is not moving, move to the item drop position.
      If the movement is successful, log a success message, wait for 1 second, publish a message to open the 
      gripper, wait for 5 seconds, reset the robot loop, move back to the lookout position, and wait for 1 second.
      If the movement is not successful, log an error message and shut down the node.*/
      // Moving to drop item position & dropping the item
      if(is_with_item_at_lookout_position && !is_moving){
        bool const item_lookout_pose_res = this->move(*item_drop_pose, "Moving to item drop position");

        if(item_lookout_pose_res){
          RCLCPP_INFO(this->get_logger(), "Arrived at item drop position.");
          rclcpp::sleep_for(1s);
          publisher->publish(std_msgs::msg::String().set__data("open"));
          rclcpp::sleep_for(5s);
          reset_robot_loop();
          this->move_to_lookout_position();
          target_pose = *lookout_pos;
          rclcpp::sleep_for(1s);
        }
        else {
          RCLCPP_INFO(this->get_logger(), "Could not arrive at item drop position. Shutting down.");
          rclcpp::shutdown();
        }
      }
    }

    /*This function is a member function of the RobotMasterController class. It is responsible for moving the robot to a 
    predefined "lookout position."*/
    void move_to_lookout_position(){
      RCLCPP_INFO(this->get_logger(), "=======================================================");
      bool const move_res = move(*lookout_pos, "Moving to lookout position");
      if(move_res){
        is_lookout_position = true;
        RCLCPP_INFO(this->get_logger(), "Arrived at lookout position.");
      }
      else {
        RCLCPP_INFO(this->get_logger(), "Could not arrive at lookout position. Shutting down.");
        rclcpp::shutdown();
      }

    }

    /*This function is designed to reset the state of the RobotMasterController object and return the robot to its initial 
    lookout position. It resets various state variables of the robot controller and moves the robot back to the lookout 
    position.*/
    void reset_robot_loop(){
      is_lookout_position = false;
      is_horizontally_centered = false;
      is_vertically_centered = false;
      is_moving = false;
      is_depth_reached = false;
      is_at_item_position = false;
      is_item_grabbed = false;
      is_item_picked = false;
      is_with_item_at_lookout_position = false;
      was_centered_message_shown = false;
      /*depths is presumably a container (e.g., std::vector) holding depth measurements. clear() empties this container, 
      resetting any stored depth data.*/
      depths.clear();
      /*prev_x keeps track of the previous horizontal position. Reset to 0 to clear any previous position data.*/
      prev_x = 0;
      this->move_to_lookout_position();
    }

    /*This function takes a raw depth value as a string, converts it to a floating-point number, normalizes it, 
    and then clamps it within a specific range. This function is a member function of the RobotMasterController class.
It takes a single argument, raw_depth, which is a string representation of the depth value.*/
    float sanitize_depth(std::string raw_depth){
      float depth = std::stof(raw_depth) / 1000;
      if(depth > 0.8){
        depth = 0.8;
      }
      else if(depth < 0){
        depth = 0;
      }
      return depth;
      /*If depth is greater than 0.8, it is set to 0.8. This ensures the depth does not exceed 0.8 meters */
    }

    /*This code defines two overloaded move functions for moving a robot using the MoveIt! library in ROS 2. 
    These functions are responsible for planning and executing a Cartesian path to reach a target pose or a series 
    of target poses. */

    /*Single Pose move Function*/

    bool move(geometry_msgs::msg::Pose target_pose, const char * log_message = "Moving robot"){
      is_moving = true;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      /*moveit::planning_interface::MoveGroupInterface::Plan my_plan;: Creates a plan object to store the trajectory*/
      move_group_->setEndEffectorLink("wrist_3_link");
      /*Sets the end-effector link for the robot's manipulator.*/

      double eef_step = 0.01; // Trajectory resolution
      /*Sets the resolution of the trajectory, defining the step size for the Cartesian path.*/
      auto res = move_group_->computeCartesianPath(std::vector<geometry_msgs::msg::Pose> {target_pose}, eef_step, 0.0, my_plan.trajectory_);
      /*Computes the Cartesian path to the target pose. Sets eef_step to 0.01. Calls computeCartesianPath with a single
      target_pose (wrapped in a vector).*/
      RCLCPP_INFO(this->get_logger(), log_message);

      /*logging and execution*/
      /*Checks if the planning was successful*/
      if (res != -1) {
        auto move_res = move_group_->execute(my_plan);
        /*Executes the planned trajectory.*/
          if(move_res == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            is_moving = false;
            RCLCPP_INFO(this->get_logger(), "Execution successful for the waypoint.");
            return true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Execution failed for the waypoint.");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan the trajectory");
        }
        is_moving = false;
        return false;
    }

    /*Multiple Poses move Function*/

    bool move(std::vector<geometry_msgs::msg::Pose> target_poses, const char * log_message = "Moving robot"){
      is_moving = true;
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      move_group_->setEndEffectorLink("wrist_3_link");

      double eef_step = 0.01; // Rozdzielczość trajektorii
      auto res = move_group_->computeCartesianPath(target_poses, eef_step, 0.0, my_plan.trajectory_);
      /*Computes the Cartesian path to a series of target poses. Sets eef_step to 0.01. Calls computeCartesianPath with 
      a vector of target_poses. */
      RCLCPP_INFO(this->get_logger(), log_message);

      /*Same as the single pose move function, but operates on a series of target poses instead of a single target pose.*/
      if (res != -1) {
        auto move_res = move_group_->execute(my_plan);
          if(move_res == moveit::planning_interface::MoveItErrorCode::SUCCESS){
            is_moving = false;
            RCLCPP_INFO(this->get_logger(), "Execution successful for the waypoint.");
            return true;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Execution failed for the waypoint.");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Failed to plan the trajectory");
        }
        is_moving = false;
        return false;
    }

    rclcpp::Subscription<ur_custom_interfaces::msg::URCommand>::SharedPtr subscription_;
    bool is_lookout_position;
    bool is_horizontally_centered;
    bool is_vertically_centered;
    bool is_moving;
    bool is_depth_reached;
    bool is_at_item_position;
    bool is_item_grabbed;
    bool is_item_picked;
    bool is_with_item_at_lookout_position;
    int prev_x;
    bool was_centered_message_shown;
    float depth;  /*Stores the current depth (distance) value. */
    std::vector<float> depths; /*A vector of float values representing multiple depth measurements.
    This might be used for averaging depths or other calculations.*/
    rclcpp::Time end_timer;  /*Stores the end time for a timer. This might be used for timing operations or delays.*/
    rclcpp::Time timer;  /*Stores the start time for a timer. This might be used for timing operations or delays.*/
    geometry_msgs::msg::Pose* lookout_pos;
    geometry_msgs::msg::Pose* item_drop_pose;
    moveit::planning_interface::MoveGroupInterface* move_group_;
    geometry_msgs::msg::Pose target_pose; /*Represents the target pose (position and orientation) the robot should move to.*/
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    std::vector<geometry_msgs::msg::Pose> waypoints;  /*A vector of poses representing waypoints. These are used for moving 
    the robot through a sequence of positions.*/
};

int main(int argc, char * argv[])
{

    geometry_msgs::msg::Pose lookout_pos;
  lookout_pos.orientation.w = 0.00029;
  lookout_pos.orientation.x = 0.902654;
  lookout_pos.orientation.y = 0.430145;
  lookout_pos.orientation.z = -0.0128905;
  lookout_pos.position.x = 0.663481;
  lookout_pos.position.y = 0.392979;
  lookout_pos.position.z = 0.616589;

    geometry_msgs::msg::Pose item_drop_pos;
  item_drop_pos.orientation.w = -0.005963;
  item_drop_pos.orientation.x = 0.751809;
  item_drop_pos.orientation.y = 0.659346;
  item_drop_pos.orientation.z = 0.002643;
  item_drop_pos.position.x = 0.379865;
  item_drop_pos.position.y = 0.753293;
  item_drop_pos.position.z = 0.532143;


  rclcpp::init(argc, argv); /*initializes the ROS 2 system with command-line arguments.*/
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto move_robot_node = rclcpp::Node::make_shared("move_robot", node_options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_robot_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  rclcpp::spin(std::make_shared<RobotMasterController>(move_robot_node, &lookout_pos, &item_drop_pos));
  rclcpp::shutdown();
  return 0;
}  
