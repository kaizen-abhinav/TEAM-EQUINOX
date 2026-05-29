#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "aeb_fuzzy/fuzzy_brake.hpp"

using std::placeholders::_1;

class AEBNode : public rclcpp::Node
{
public:
  AEBNode() : Node("aeb_fuzzy_node")
  {
    // Subscribers
    dist_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/aeb/distance", 10,
      std::bind(&AEBNode::on_distance, this, _1));

    closing_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/aeb/closing_speed", 10,
      std::bind(&AEBNode::on_closing, this, _1));

    ego_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/aeb/ego_speed", 10,
      std::bind(&AEBNode::on_ego, this, _1));

    // Publisher
    brake_pub_ = create_publisher<std_msgs::msg::Float32>(
      "/aeb/brake_cmd", 10);

    // Timer (10 Hz)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AEBNode::loop, this));

    RCLCPP_INFO(get_logger(), "AEB Fuzzy Node started");
  }

private:
  /* ===================== CALLBACKS ===================== */

  void on_distance(const std_msgs::msg::Float32::SharedPtr msg)
  {
    distance_ = msg->data;
    got_distance_ = true;
  }

  void on_closing(const std_msgs::msg::Float32::SharedPtr msg)
  {
    closing_speed_ = msg->data;
    got_closing_ = true;
  }

  void on_ego(const std_msgs::msg::Float32::SharedPtr msg)
  {
    ego_speed_ = msg->data;
    got_ego_ = true;
  }

  /* ===================== MAIN LOOP ===================== */

  void loop()
  {
    if (!(got_distance_ && got_closing_ && got_ego_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for AEB inputs...");
      return;
    }

    float brake = fuzzy_.compute_brake(
      distance_,
      closing_speed_,
      ego_speed_);

    // 🔴 PHASE-4 INSTRUMENTATION LOG 🔴
    RCLCPP_INFO(
      get_logger(),
      "AEB | dist=%.2f m | closing=%.2f m/s | ego=%.2f km/h | brake=%.3f",
      distance_,
      closing_speed_,
      ego_speed_,
      brake);

    std_msgs::msg::Float32 out;
    out.data = brake;
    brake_pub_->publish(out);
  }

  /* ===================== MEMBERS ===================== */

  // ROS
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr closing_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ego_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data
  float distance_ = 0.0f;
  float closing_speed_ = 0.0f;
  float ego_speed_ = 0.0f;

  bool got_distance_ = false;
  bool got_closing_ = false;
  bool got_ego_ = false;

  // Fuzzy controller
  FuzzyBrakeController fuzzy_;
};

/* ===================== MAIN ===================== */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBNode>());
  rclcpp::shutdown();
  return 0;
}
