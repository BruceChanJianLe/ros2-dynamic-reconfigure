#ifndef ROS2_DYNAMIC_RECONFIGURE_TALKER_HPP
#define ROS2_DYNAMIC_RECONFIGURE_TALKER_HPP

// STL
#include <memory>
#include <string>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>

namespace dynamic_reconfigure
{
  class Talker
  {
  public:
    using Ptr = std::shared_ptr<Talker>;
    explicit Talker(rclcpp::Node::SharedPtr n);
    ~Talker();

  private:
    rclcpp::Node::SharedPtr n_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void run();

    // Variables
    int  rate_;
    int data_;
    int ns_data_;
    rclcpp::Logger logger_;
    double a_double;
    double a_int;
    double a_bool;
    std::string a_string;

    void getParams();
    /// \brief Dynamic parameters callback
    rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    /// \brief Dynamic parameters handler
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    /// \brief Print param update for user
    void printParamUpdate(const std::string &name, const std::string &value) const;
  };

} // namespace dynamic_reconfigure

#endif /* ROS2_DYNAMIC_RECONFIGURE_TALKER_HPP */
