#include "ros2-dynamic-reconfigure/talker.hpp"

namespace dynamic_reconfigure
{
  Talker::Talker(rclcpp::Node::SharedPtr n)
  : n_{n}
  , logger_{n_->get_logger()}
  {
    getParams();
    pub_ = n_->create_publisher<std_msgs::msg::Int16>("listen_here", 1);
    timer_ = n_->create_wall_timer(
        std::chrono::seconds(rate_),
        [this]()
        {
          this->run();
        });

    dyn_params_handler_ = n_->add_on_set_parameters_callback(
      [this](std::vector<rclcpp::Parameter> parameters)
      {
        return this->dynamicParametersCallback(parameters);
      }
    );
  }

  Talker::~Talker()
  {
  }

  void Talker::run()
  {
    // Prepare msg
    auto msg = std_msgs::msg::Int16();
    msg.data = data_;

    pub_->publish(msg);

    // Print in terminal
    RCLCPP_INFO_STREAM(logger_, "I have spoken " << msg.data << " to the listen_here topic.");
  }

  void Talker::getParams()
  {
    n_->declare_parameter("rate",     rclcpp::ParameterValue(1));
    n_->declare_parameter("data",     rclcpp::ParameterValue(0));
    n_->declare_parameter("namespace1.data",     rclcpp::ParameterValue(0));
    n_->declare_parameter("a_double", rclcpp::ParameterValue(0.0));
    n_->declare_parameter("a_int",    rclcpp::ParameterValue(0.0));
    n_->declare_parameter("a_bool",   rclcpp::ParameterValue(false));
    n_->declare_parameter("a_string", rclcpp::ParameterValue(""));
    n_->get_parameter("rate", rate_);
    n_->get_parameter("data", data_);
  }

  rcl_interfaces::msg::SetParametersResult Talker::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    bool unknown_param = false;
    RCLCPP_INFO_STREAM(logger_, "Dynamic Reconfigure Callback for " << n_->get_name());

    try
    {
      // Update parameters
      for (auto parameter : parameters)
      {
        const auto &type = parameter.get_type();
        const auto &name = parameter.get_name();

        // Sanity check, need not do anything if it's not the local filter namespace
        // if (name.find(namespace_) == std::string::npos)
        // {
        //   continue;
        // }

        switch (type)
        {
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          if (name.compare("a_double") == 0)
            this->a_double = parameter.as_double();
          else
            unknown_param = true;
          break;

        case rclcpp::ParameterType::PARAMETER_STRING:
          if (name.compare("a_string") == 0)
            this->a_string = parameter.as_string();
          else
            unknown_param = true;
          break;

        case rclcpp::ParameterType::PARAMETER_BOOL:
          if (name.compare("a_bool") == 0)
            this->a_bool = parameter.as_bool();
          else
            unknown_param = true;
          break;

        case rclcpp::ParameterType::PARAMETER_INTEGER:
          if (name.compare("data") == 0)
            this->data_ = parameter.as_int();
          if (name.compare("namespace1.data") == 0)
            this->ns_data_ = parameter.as_int();
          else
            unknown_param = true;
          break;

        default:
          unknown_param = true;
          break;
        }

        if (unknown_param)
          RCLCPP_WARN_STREAM(
              this->logger_,
              "("
                  << __func__
                  << ") unknown param - "
                  << name
                  << " was not set with "
                  << parameter.value_to_string());
        else
          this->printParamUpdate(name.c_str(), parameter.value_to_string());
      }
    }
    catch (const rclcpp::ParameterTypeException &e)
    {
      RCLCPP_ERROR_STREAM(
          this->logger_,
          "("
              << __func__
              << ") caught an error - "
              << e.what());

      result.successful = false;
      result.reason = "ParameterTypeException";
    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(
          this->logger_,
          "("
              << __func__
              << ") caught an error.");

      result.successful = false;
      result.reason = "Some error";
    }

    if (unknown_param)
    {
      result.successful = false;
      result.reason = "Could not set an unknown param";
    }
    else
    {
      result.successful = true;
      result.reason = "success";
    }

    return result;
  }

  void Talker::printParamUpdate(const std::string &name, const std::string &value) const
  {
    RCLCPP_INFO_STREAM(
        this->logger_,
        name
            << " is updated with "
            << value
            << ".");
  }
} // namespace dynamic_reconfigure