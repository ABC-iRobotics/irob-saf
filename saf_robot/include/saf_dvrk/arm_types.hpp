// include/saf_dvrk/arm_types.hpp
#pragma once

#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace saf
{

    enum class ArmType
    {
        PSM,
        MTM,
        ECM,
        Unknown
      };

    inline ArmType from_string(const std::string &name)
    {
        if (name == "PSM" || name == "psm") return ArmType::PSM;
        if (name == "MTM" || name == "mtm") return ArmType::MTM;
        if (name == "ECM" || name == "ecm") return ArmType::ECM;
        return ArmType::Unknown;
    }

    inline std::string to_string(ArmType t)
    {
        switch (t)
        {
        case ArmType::PSM: return "PSM";
        case ArmType::MTM: return "MTM";
        case ArmType::ECM: return "ECM";
        default: return "Unknown";
        }
    }

    struct ArmState
    {
        geometry_msgs::msg::PoseStamped pose;
        sensor_msgs::msg::JointState joints;
        bool connected{false};
    };

}  // namespace saf
