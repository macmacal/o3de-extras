#include "DraftArticulationForcePublisher.h"
#include <AzCore/Serialization/EditContext.h>

#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Conversions.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Utilities/ArticulationsUtilities.h>
#include "PhysX/ArticulationSensorBus.h"
namespace ROS2
{
    const char* WrenchStampedType = "geometry_msgs::msg::WrenchStamped";
    DraftArticulationForcePublisher::DraftArticulationForcePublisher()
    {

        TopicConfiguration tc;

        tc.m_type = WrenchStampedType;
        tc.m_topic = "wrench";
        m_sensorConfiguration.m_frequency = 15;
        m_sensorConfiguration.m_publishersConfigurations.insert(AZStd::make_pair(WrenchStampedType, AZStd::move(tc)));

    }

    void DraftArticulationForcePublisher::Activate() {
        auto ros2Node = ROS2Interface::Get()->GetNode();
        AZ_Assert(ros2Node, "ROS2 node is not available");
        AZ_Assert(m_sensorConfiguration.m_publishersConfigurations.contains(WrenchStampedType), "Invalid configuration of publishers for Draft Articulation Force Publisher");
        const auto publisherConfig = m_sensorConfiguration.m_publishersConfigurations[WrenchStampedType];
        const auto fullTopic = ROS2Names::GetNamespacedName(GetNamespace(), publisherConfig.m_topic);
        m_wrenchPublisher = ros2Node->create_publisher<geometry_msgs::msg::WrenchStamped>(fullTopic.data(), publisherConfig.GetQoS());
        ROS2SensorComponentBase<PhysicsBasedSource>::Activate();
        StartSensor(
                m_sensorConfiguration.m_frequency,
                [this](float imuDeltaTime, AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime)
                {
                    if (!m_sensorConfiguration.m_publishingEnabled)
                    {
                        return;
                    }
                    OnPhysicsEvent();
                },
                []([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float physicsDeltaTime)
                {

                });

    }

    void DraftArticulationForcePublisher::Deactivate() {
        ROS2SensorComponentBase<PhysicsBasedSource>::Deactivate();
    }


    void DraftArticulationForcePublisher::OnPhysicsEvent() {

        AZ::Vector3 force = AZ::Vector3(-1.0f, 0.0f, 0.0f);
        AZ::Vector3 torque = AZ::Vector3(-1.0f, 0.0f, 0.0f);
        PhysX::ArticulationSensorRequestBus::EventResult(force, GetEntityId(), &PhysX::ArticulationSensorRequests::GetForce, 0);
        PhysX::ArticulationSensorRequestBus::EventResult(torque, GetEntityId(), &PhysX::ArticulationSensorRequests::GetTorque, 0);
        geometry_msgs::msg::WrenchStamped wrench;

        wrench.header.frame_id = GetFrameID().c_str();
        wrench.header.stamp = ROS2Interface::Get()->GetROSTimestamp();
        wrench.wrench.force.x = force.GetX();
        wrench.wrench.force.y = force.GetY();
        wrench.wrench.force.z = force.GetZ();
        wrench.wrench.torque.x = torque.GetX();
        wrench.wrench.torque.y = torque.GetY();
        wrench.wrench.torque.z = torque.GetZ();

        AZ_Assert(m_wrenchPublisher, "Wrench publisher is not available");
        m_wrenchPublisher->publish(wrench);

    }

    void DraftArticulationForcePublisher::Reflect(AZ::ReflectContext* context) {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context)) {
            serialize->Class<DraftArticulationForcePublisher, SensorBaseType>()->Version(1);
            if (auto* editContext = serialize->GetEditContext()) {
                editContext->Class<DraftArticulationForcePublisher>("Draft Articulation Force Publisher", "Draft Articulation Force Publisher")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2-Experimental")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"));
            }
        }
    }
}