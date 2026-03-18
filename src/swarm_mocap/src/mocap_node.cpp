#include <string>
#include <vector>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "swarm_mocap/owl.hpp"

// ---------------------------------------------------------------------------
// Coordinate convention — matches the lab's existing ROS1 node:
//   OWL native: x, y, z in millimetres; quaternion [s, x, y, z]
//   Published:  x = owl_x / 1000
//               y = -owl_z / 1000   (axis swap)
//               z =  owl_y / 1000   (axis swap)
// ---------------------------------------------------------------------------

class MoCapNode : public rclcpp::Node
{
public:
  MoCapNode() : Node("swarm_mocap")
  {
    // Parameters
    declare_parameter("server_ip",           std::string("192.168.1.71"));
    declare_parameter("frame_id",            std::string("mocap"));
    // IDs of rigid bodies to publish individually on /mocap/rigid_{id}
    declare_parameter("published_rigid_ids", std::vector<int64_t>{});

    server_ip_   = get_parameter("server_ip").as_string();
    frame_id_    = get_parameter("frame_id").as_string();
    auto ids     = get_parameter("published_rigid_ids").as_integer_array();
    for (auto id : ids) published_rigid_ids_.insert(static_cast<uint32_t>(id));

    // Publishers
    rigids_pub_  = create_publisher<geometry_msgs::msg::PoseArray>("/mocap/rigids",  10);
    markers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/mocap/markers", 10);

    RCLCPP_INFO(get_logger(), "Connecting to PhaseSpace OWL server at %s ...",
                server_ip_.c_str());

    if (owl_.open(server_ip_) <= 0 || owl_.initialize("timebase=1,1000000") <= 0) {
      RCLCPP_ERROR(get_logger(),
                   "OWL connection failed — no MoCap server at %s\n"
                   "  Start the PhaseSpace server and set server_ip correctly.",
                   server_ip_.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Connected. Streaming started.");
    owl_.streaming(1);

    // Poll loop driven by a ROS2 timer so SIGINT is handled cleanly
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&MoCapNode::poll, this));
  }

  ~MoCapNode()
  {
    if (owl_.isOpen()) {
      owl_.done();
      owl_.close();
    }
  }

private:
  void poll()
  {
    const OWL::Event *event = owl_.nextEvent(0);  // non-blocking
    if (!event) return;

    if (event->type_id() == OWL::Type::ERROR) {
      RCLCPP_WARN(get_logger(), "OWL error: %s", event->str().c_str());
      return;
    }

    if (event->type_id() != OWL::Type::FRAME) return;

    auto stamp = now();

    // --- Rigid bodies ---
    OWL::Rigids rigids;
    if (event->find("rigids", rigids) > 0) {
      auto array_msg = geometry_msgs::msg::PoseArray();
      array_msg.header.stamp    = stamp;
      array_msg.header.frame_id = frame_id_;

      for (const auto &r : rigids) {
        if (r.cond <= 0.0f) continue;  // not tracked

        auto pose = to_pose(r.pose);
        array_msg.poses.push_back(pose);

        // Per-rigid PoseStamped for direct subscription by controllers
        if (published_rigid_ids_.count(r.id)) {
          auto ps = geometry_msgs::msg::PoseStamped();
          ps.header = array_msg.header;
          ps.pose   = pose;
          per_rigid_pub(r.id)->publish(ps);
        }
      }
      rigids_pub_->publish(array_msg);
    }

    // --- Markers ---
    OWL::Markers markers;
    if (event->find("markers", markers) > 0) {
      auto array_msg = geometry_msgs::msg::PoseArray();
      array_msg.header.stamp    = stamp;
      array_msg.header.frame_id = frame_id_;

      for (const auto &m : markers) {
        if (m.cond <= 0.0f) continue;

        geometry_msgs::msg::Pose pose;
        pose.position.x    = m.x / 1000.0f;
        pose.position.y    = -m.z / 1000.0f;
        pose.position.z    = m.y / 1000.0f;
        pose.orientation.w = 1.0;  // markers have no orientation
        array_msg.poses.push_back(pose);
      }
      markers_pub_->publish(array_msg);
    }
  }

  // Convert OWL rigid pose[7] = [x,y,z, qw,qx,qy,qz] (mm) to ROS geometry_msgs::Pose
  static geometry_msgs::msg::Pose to_pose(const float pose[7])
  {
    geometry_msgs::msg::Pose p;
    p.position.x    =  pose[0] / 1000.0f;
    p.position.y    = -pose[2] / 1000.0f;
    p.position.z    =  pose[1] / 1000.0f;
    p.orientation.w =  pose[3];
    p.orientation.x =  pose[4];
    p.orientation.y = -pose[6];
    p.orientation.z =  pose[5];
    return p;
  }

  // Lazily create one PoseStamped publisher per rigid ID
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr per_rigid_pub(uint32_t id)
  {
    auto it = per_rigid_pubs_.find(id);
    if (it != per_rigid_pubs_.end()) return it->second;

    std::string topic = "/mocap/rigid_" + std::to_string(id);
    auto pub = create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
    per_rigid_pubs_[id] = pub;
    RCLCPP_INFO(get_logger(), "Publishing rigid body %u on %s", id, topic.c_str());
    return pub;
  }

  // State
  OWL::Context owl_;
  std::string server_ip_;
  std::string frame_id_;
  std::unordered_set<uint32_t> published_rigid_ids_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr rigids_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr markers_pub_;
  std::unordered_map<uint32_t,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> per_rigid_pubs_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoCapNode>());
  rclcpp::shutdown();
  return 0;
}
