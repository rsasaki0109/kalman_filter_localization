#include <kalman_filter_localization/ekf_localization_component.h>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<kalman_filter_localization::EkfLocalizationComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}