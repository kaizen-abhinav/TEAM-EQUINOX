#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "endurance/msg/radar_track_list.hpp"
#include "endurance/msg/velocity.hpp"
#include "aeb_fuzzy/fuzzy_brake.hpp"
#include <cmath>
#include <limits>

using std::placeholders::_1;

class AEBNode : public rclcpp::Node
{
public:
  AEBNode() : Node("aeb_fuzzy_node")
  {
    // Subscribers
    radar_sub_ = create_subscription<endurance::msg::RadarTrackList>(
      "/RadarObjects", 10,
      std::bind(&AEBNode::on_radar, this, _1));

    ego_sub_ = create_subscription<endurance::msg::Velocity>(
      "/VehicleSpeed", 10,
      std::bind(&AEBNode::on_ego, this, _1));

    // Publishers
    brake_pub_ = create_publisher<std_msgs::msg::Float32>("/aeb/brake_cmd", 10);
    dist_pub_ = create_publisher<std_msgs::msg::Float32>("/aeb/distance", 10);
    closing_pub_ = create_publisher<std_msgs::msg::Float32>("/aeb/closing_speed", 10);
    ego_pub_ = create_publisher<std_msgs::msg::Float32>("/aeb/ego_speed", 10);

    // Timer (20 Hz)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AEBNode::loop, this));

    RCLCPP_INFO(get_logger(), "AEB Fuzzy Node (Advanced) started");
  }

private:
  /* ===================== CALLBACKS ===================== */

  void on_radar(const endurance::msg::RadarTrackList::SharedPtr msg)
  {
    float best_ttc = std::numeric_limits<float>::max();
    float best_dist = 300.0f;
    float best_closing = 0.0f;
    bool found_target = false;

    for (const auto& obj : msg->objects)
    {
      float dist_x = std::abs(obj.x_distance);
      float dist_y = obj.y_distance;
      float vrel_x = obj.vx; // Negative means closing in
      float vrel_y = obj.vy;

      if (dist_x < 0.1f || dist_x > 300.0f) continue;

      bool in_path = false;

      // 1. Ego Lane Check (Vehicle width ~2m, Lane width ~3.5m)
      // WIDENED to 2.5m to instantly catch vehicles straddling the line
      if (std::abs(dist_y) < 2.5f) {
        in_path = true;
      }
      // 2. Cut-in Check (Vehicle moving towards our lane laterally)
      // LOWERED velocity threshold to catch slow cut-ins
      else if ((dist_y > 0 && vrel_y < -0.05f) || (dist_y < 0 && vrel_y > 0.05f)) {
        float ttc_y = std::abs(dist_y) / std::abs(vrel_y);
        // Predict X position when it crosses our path
        float pred_x = dist_x + vrel_x * ttc_y;
        if (ttc_y < 5.0f && pred_x > 0.0f && pred_x < 80.0f) {
          in_path = true;
        }
      }

      if (in_path) {
        found_target = true;
        float closing_speed = -vrel_x; // Positive = approaching
        
        float current_ttc = std::numeric_limits<float>::max();
        if (closing_speed > 0.1f) {
          current_ttc = dist_x / closing_speed;
        } else if (dist_x < 10.0f) {
          // If close and not moving away, treat as urgent
          current_ttc = dist_x / 0.1f; 
        }

        // Prioritize lowest TTC, fallback to closest distance
        if (current_ttc < best_ttc || (current_ttc == best_ttc && dist_x < best_dist)) {
          best_ttc = current_ttc;
          best_dist = dist_x;
          best_closing = closing_speed;
        }
      }
    }

    target_distance_ = best_dist;
    target_closing_ = best_closing;
    got_radar_ = true;
  }

  void on_ego(const endurance::msg::Velocity::SharedPtr msg)
  {
    ego_speed_kmh_ = msg->vehicle_velocity * 3.6f;
    got_ego_ = true;
  }

  /* ===================== MAIN LOOP ===================== */

  void loop()
  {
    if (!got_radar_ || !got_ego_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for advanced AEB inputs (Radar/Ego)...");
      return;
    }

    float brake = fuzzy_.compute_brake(
      target_distance_,
      target_closing_,
      ego_speed_kmh_);

    // Publish to plotter & CarMaker bridge
    std_msgs::msg::Float32 out_brake, out_dist, out_close, out_ego;
    out_brake.data = brake;
    out_dist.data = target_distance_;
    out_close.data = target_closing_;
    out_ego.data = ego_speed_kmh_;

    brake_pub_->publish(out_brake);
    dist_pub_->publish(out_dist);
    closing_pub_->publish(out_close);
    ego_pub_->publish(out_ego);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 100, // Log at 10Hz
      "AEB | dist=%.2f m | closing=%.2f m/s | ego=%.2f km/h | brake=%.3f",
      target_distance_, target_closing_, ego_speed_kmh_, brake);
  }

  /* ===================== MEMBERS ===================== */

  rclcpp::Subscription<endurance::msg::RadarTrackList>::SharedPtr radar_sub_;
  rclcpp::Subscription<endurance::msg::Velocity>::SharedPtr ego_sub_;
  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr closing_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ego_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;

  float target_distance_ = 300.0f;
  float target_closing_ = 0.0f;
  float ego_speed_kmh_ = 0.0f;

  bool got_radar_ = false;
  bool got_ego_ = false;

  FuzzyBrakeController fuzzy_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AEBNode>());
  rclcpp::shutdown();
  return 0;
}