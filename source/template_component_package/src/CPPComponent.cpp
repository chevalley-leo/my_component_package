#include "template_component_package/CPPComponent.hpp"

namespace template_component_package {
CPPComponent::CPPComponent(const rclcpp::NodeOptions& options) :
    modulo_components::Component(options, "CPPComponent") {
  // add parameters, inputs and outputs here
}

bool CPPComponent::on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>&) {
  // validate an incoming parameter value according to some criteria
  return true;
}

bool CPPComponent::on_execute_callback() {
  // If the component needs to do any post-construction behavior, invoke `execute()`
  // at the end of the constructor, which will trigger this callback in a separate thread.
  // This is only necessary when the behavior would otherwise block the constructor from completing
  // in a timely manner, such as some time-intensive computation or waiting for an external trigger.

  // return true if the execution was successful, false otherwise
  return true;
}

} // namespace template_component_package

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(template_component_package::CPPComponent)