#pragma once

#include <modulo_components/Component.hpp>

namespace my_custom_package {
class CPPComponent : public modulo_components::Component {
public:
  explicit CPPComponent(const rclcpp::NodeOptions& options);

protected:
  bool on_validate_parameter_callback(const std::shared_ptr<state_representation::ParameterInterface>& parameter) override;

  bool on_execute_callback() override;
};
}  // namespace my_custom_package