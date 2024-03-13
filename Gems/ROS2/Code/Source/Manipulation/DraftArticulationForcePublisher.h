
#pragma once

#include <ROS2/Sensor/Events/PhysicsBasedSource.h>
#include <ROS2/Sensor/ROS2SensorComponentBase.h>
#include <rclcpp/publisher.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace ROS2
{
    class DraftArticulationForcePublisher : public ROS2SensorComponentBase<PhysicsBasedSource>
    {
    public:
        AZ_COMPONENT(DraftArticulationForcePublisher, "{018e37ab-8ef1-7646-b5bb-1db910aa2bc4}", SensorBaseType);
        DraftArticulationForcePublisher();
        ~DraftArticulationForcePublisher() = default;
        static void Reflect(AZ::ReflectContext* context);
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;
    private:

        void OnPhysicsEvent();
        bool m_initialized = false;
        AZ::EntityId m_articulationRootEntity;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> m_wrenchPublisher;
    };
}// namespace ROS2