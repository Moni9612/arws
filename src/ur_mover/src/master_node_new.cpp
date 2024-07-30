#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <ur_custom_interfaces/msg/ur_command.hpp>
#include <std_msgs/msg/string.hpp>

class RobotMasterController : public rclcpp::Node {
public:
  RobotMasterController(std::shared_ptr<rclcpp::Node> move_group_node, geometry_msgs::msg::Pose* lookout_pos, geometry_msgs::msg::Pose* item_drop_pos)
  : Node("master_node"), is_lookout_position(false), is_horizontally_centered(false), 
  is_vertically_centered(false), is_moving(false), lookout_pos(lookout_pos), target_pose(*lookout_pos), prev_x(0),
  is_depth_reached(false), was_centered_message_shown(false), depth(0.0), item_drop_pose(item_drop_pos), is_at_item_position(false), 
  is_item_grabbed(false), is_item_picked(false), is_with_item_at_lookout_position(false)
  {
    RCLCPP_INFO(this->get_logger(), "Node started. Awaiting commands...");
    RCLCPP_INFO(this->get_logger(), "=======================================================");
    RCLCPP_INFO(this->get_logger(), "SETUP LOGS");
    RCLCPP_INFO(this->get_logger(), "=======================================================");

    subscription_ = this->create_subscription<ur_custom_interfaces::msg::URCommand>(
    "custom_camera", 1, std::bind(&RobotMasterController::topic_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to /custom_camera topic");

    publisher = this->create_publisher<std_msgs::msg::String>("/custom_gripper", 1);
    RCLCPP_INFO(this->get_logger(), "Publisher on custom_gripper created");

    move_group_ = new moveit::planning_interface::MoveGroupInterface(move_group_node, "ur_manipulator");
    auto const robot_pos = move_group_->getCurrentPose("wrist_3_link");
    RCLCPP_INFO(this->get_logger(), "Robot position: %f, %f, %f", robot_pos.pose.position.x, robot_pos.pose.position.y, robot_pos.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Robot rotation: %f, %f, %f, %f", 
    robot_pos.pose.orientation.x, robot_pos.pose.orientation.y, robot_pos.pose.orientation.z, robot_pos.pose.orientation.w
    );

    RCLCPP_INFO(this->get_logger(), "Lookout position: %f, %f, %f", lookout_pos->position.x, lookout_pos->position.y, lookout_pos->position.z);
    this->move_to_lookout_position();
  }

  void topic_callback(const ur_custom_interfaces::msg::URCommand::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Callback triggered");
    int x = std::stoi(msg->x);
    int y = std::stoi(msg->y);
    RCLCPP_DEBUG(this->get_logger(), "Received command: x=%d, y=%d, depth=%s", x, y, msg->depth.c_str());

    if(!is_moving) {
      depth = sanitize_depth(msg->depth);
      RCLCPP_DEBUG(this->get_logger(), "Sanitized depth: %f", depth);
    }

    if(depth > 0.0 && depth < 0.7) {
      depths.push_back(depth);
      RCLCPP_DEBUG(this->get_logger(), "Depth added to depths list: %f", depth);
    }

    if(is_depth_reached && is_horizontally_centered && is_vertically_centered) {
      RCLCPP_DEBUG(this->get_logger(), "Already at item position");
      return;
    }

    if(is_moving || !is_lookout_position) {
      RCLCPP_DEBUG(this->get_logger(), "Robot is moving or not at lookout position. Ignoring command.");
      return;
    }

    if(x == 0 && !is_horizontally_centered) {
      RCLCPP_DEBUG(this->get_logger(), "Robot centered horizontally");
      is_horizontally_centered = true;
      is_moving = false;
      move_group_->stop();
      return;
    }

    if(x == 1 && !is_moving && !is_horizontally_centered) {
      target_pose.position.x += 0.01;
      RCLCPP_DEBUG(this->get_logger(), "Moving robot to the right. New target pose: (%f, %f, %f)", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      this->move(target_pose, "Moving robot to the right");
    } else if(x == -1 && !is_moving && !is_horizontally_centered) {
      target_pose.position.x -= 0.01;
      RCLCPP_DEBUG(this->get_logger(), "Moving robot to the left. New target pose: (%f, %f, %f)", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      this->move(target_pose, "Moving robot to the left");
    }

    if(!is_horizontally_centered) {
      RCLCPP_DEBUG(this->get_logger(), "Robot is not centered horizontally. Ignoring command.");
      return;
    }

    if(y == 0 && !is_vertically_centered) {
      RCLCPP_DEBUG(this->get_logger(), "Robot centered vertically");
      is_vertically_centered = true;
      move_group_->stop();
      is_moving = false;
      return;
    }

    if(y == 1 && !is_moving && !is_vertically_centered) {
      target_pose.position.z -= 0.01;
      RCLCPP_DEBUG(this->get_logger(), "Moving robot to the bottom. New target pose: (%f, %f, %f)", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      this->move(target_pose, "Moving robot to the bottom");
    } else if(y == -1 && !is_moving && !is_vertically_centered) {
      target_pose.position.z += 0.01;
      RCLCPP_DEBUG(this->get_logger(), "Moving robot to the top. New target pose: (%f, %f, %f)", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      this->move(target_pose, "Moving robot to the top");
    }

    bool const is_robot_centered = is_horizontally_centered && is_vertically_centered;
    if(is_robot_centered && !was_centered_message_shown) {
      RCLCPP_INFO(this->get_logger(), "Robot is centered. Started timer.");
      was_centered_message_shown = true;
      end_timer = this->get_clock()->now() + rclcpp::Duration(3s);
    } else if(!is_robot_centered) {
      return;
    }

    timer = this->get_clock()->now();
    RCLCPP_DEBUG(this->get_logger(), "Current time: %ld, End time: %ld", timer.nanoseconds(), end_timer.nanoseconds());

    if(timer < end_timer) {
      RCLCPP_DEBUG(this->get_logger(), "Timer not reached. Waiting...");
      return;
    }

    if(depth < 0.01) {
      RCLCPP_DEBUG(this->get_logger(), "Depth too small. Awaiting another reading.");
      return;
    }
    if (depth > 0.8) {
      RCLCPP_DEBUG(this->get_logger(), "Depth too big. Awaiting another reading.");
      return;
    }

    if(!is_moving && !is_depth_reached) {
      target_pose.position.z += 0.06;
      RCLCPP_DEBUG(this->get_logger(), "Applying camera offset. New target pose: (%f, %f, %f)", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      bool const offset_res = this->move(target_pose, "Applying camera offset");
      if(offset_res) {
        RCLCPP_INFO(this->get_logger(), "Applied camera offset");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not apply camera offset. Shutting down.");
        rclcpp::shutdown();
      }

      RCLCPP_INFO(this->get_logger(), "Moving robot forward by %f", depth);
      float camera_offset = 0.03;
      float gripper_offset = 0.00;
      target_pose.position.y += depth - camera_offset - gripper_offset;
      target_pose.position.x -= 0.03;
      RCLCPP_DEBUG(this->get_logger(), "New target pose for forward movement: (%f, %f, %f)", 
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
      bool const forward_res = this->move(target_pose, "Moving robot forward");

      if(forward_res) {
        is_depth_reached = true;
        RCLCPP_INFO(this->get_logger(), "Arrived at item position.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not arrive at item position. Shutting down.");
        rclcpp::shutdown();
      }
    }

    if(is_depth_reached && !is_item_grabbed) {
      RCLCPP_INFO(this->get_logger(), "About to close gripper");
      this->gripper_command("close");
      is_item_grabbed = true;
    }

    if(is_depth_reached && is_item_grabbed && !is_item_picked) {
      target_pose.position.z += 0.06;
      bool const lift_res = this->move(target_pose, "Lifting item up");

      if(lift_res) {
        is_item_picked = true;
        RCLCPP_INFO(this->get_logger(), "Item lifted up.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not lift the item up. Shutting down.");
        rclcpp::shutdown();
      }
    }

    if(is_depth_reached && is_item_grabbed && is_item_picked && !is_with_item_at_lookout_position) {
      is_lookout_position = false;
      bool const back_res = this->move_to_lookout_position();

      if(back_res) {
        is_with_item_at_lookout_position = true;
        RCLCPP_INFO(this->get_logger(), "Returned to lookout position with item.");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not return to lookout position with item. Shutting down.");
        rclcpp::shutdown();
      }
    }

    if(is_with_item_at_lookout_position) {
      bool const item_res = this->move(*item_drop_pose, "Move item to drop position");

      if(item_res) {
        RCLCPP_INFO(this->get_logger(), "Moved item to drop position.");
        this->gripper_command("open");
        is_with_item_at_lookout_position = false;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Could not move item to drop position. Shutting down.");
        rclcpp::shutdown();
      }
    }
  }

  float sanitize_depth(const std::string& depth_str) {
    try {
      return std::stof(depth_str);
    } catch(const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Could not convert depth to float: %s", e.what());
      return 0.0;
    }
  }

  void gripper_command(const std::string& command) {
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = command;
    publisher->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Gripper command sent: %s", command.c_str());
  }

  bool move(geometry_msgs::msg::Pose const& target_pose, std::string const& message) {
    is_moving = true;
    move_group_->setPoseTarget(target_pose);
    auto const [success, plan] = [&] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      return std::make_pair(move_group_->plan(msg) == moveit::planning_interface::MoveItErrorCode::SUCCESS, msg);
    }();

    if(!success) {
      RCLCPP_ERROR(this->get_logger(), "Plan could not be computed: %s", message.c_str());
      is_moving = false;
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Executing plan: %s", message.c_str());
    auto const result = move_group_->execute(plan);
    is_moving = false;
    return result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
  }

  bool move_to_lookout_position() {
    return move(*lookout_pos, "Moving to lookout position");
  }

private:
  bool is_lookout_position;
  bool is_horizontally_centered;
  bool is_vertically_centered;
  bool is_moving;
  bool is_depth_reached;
  bool was_centered_message_shown;
  bool is_at_item_position;
  bool is_item_grabbed;
  bool is_item_picked;
  bool is_with_item_at_lookout_position;

  geometry_msgs::msg::Pose* lookout_pos;
  geometry_msgs::msg::Pose target_pose;
  geometry_msgs::msg::Pose* item_drop_pose;
  float depth;
  float prev_x;
  std::vector<float> depths;

  rclcpp::Subscription<ur_custom_interfaces::msg::URCommand>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  moveit::planning_interface::MoveGroupInterface* move_group_;
  rclcpp::Time end_timer;
  rclcpp::Time timer;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto const move_group_node = std::make_shared<rclcpp::Node>("move_group_node");
  geometry_msgs::msg::Pose lookout_pose, item_drop_pose;

  // Initialize lookout_pose and item_drop_pose with your desired values
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
  

  auto const master_node = std::make_shared<RobotMasterController>(move_group_node, &lookout_pose, &item_drop_pose);
  rclcpp::spin(master_node);
  rclcpp::shutdown();

  return 0;
}
