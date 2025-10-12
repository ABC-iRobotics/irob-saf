#include <irob_vision_support/vision_server.hpp>
#include <irob_vision_support/dummy_image_processor.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // VisionServer with DummyImageProcessor
    using Server = irob_vision::VisionServer<irob_vision::DummyImageProcessor>;
    auto node = std::make_shared<Server>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
