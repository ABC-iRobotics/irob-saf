/*
 * 	arm_types.hpp
 *
 *	Author(s): Tamas Levendovics
 *
 *
 */

#ifndef DVRK_ARM_PARAMS_
#define DVRK_ARM_PARAMS_

#include <iostream>
#include <string>
#include <vector>
#include <irob_utils/tool_pose.hpp>

namespace saf {

    class ArmTypes {
    public:
        // Enum value DECLARATIONS - they are defined later
        static const ArmTypes MTML;
        static const ArmTypes MTMR;
        static const ArmTypes PSM1;
        static const ArmTypes PSM2;
        static const ArmTypes PSM3;
        static const ArmTypes ECM;

        // Attributes
        const std::string name;
        const int dof;
        const ToolPose::Distance maxVelPose; // unit/sec
        const std::vector<double> maxVelJoint; // unit/sec

    private:
        ArmTypes( std::string name, int dof,
                  ToolPose::Distance maxVelPose, std::vector<double> maxVelJoint):
          name(name), dof(dof),
          maxVelPose(maxVelPose),
          maxVelJoint(maxVelJoint) { }

    public:
        static const ArmTypes typeForString(const std::string& name)
        {
            if (name == MTML.name)
                return MTML;
            if (name == MTMR.name)
                return MTMR;
            if (name == PSM1.name)
                return PSM1;
            if (name == PSM2.name)
                return PSM2;
            if (name == PSM3.name)
                return PSM3;
            if (name == ECM.name)
                return ECM;
            return PSM1;
        }

        bool operator==(const ArmTypes& other) const
        {
            return name == other.name;
        }
    };

    // Static initialization of ArmTypes
    const ArmTypes ArmTypes::MTML("MTML", 7, ToolPose::Distance(1.0), {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    const ArmTypes ArmTypes::MTMR("MTMR", 7, ToolPose::Distance(1.0), {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    const ArmTypes ArmTypes::PSM1("PSM1", 7, ToolPose::Distance(1.0), {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    const ArmTypes ArmTypes::PSM2("PSM2", 7, ToolPose::Distance(1.0), {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    const ArmTypes ArmTypes::PSM3("PSM3", 7, ToolPose::Distance(1.0), {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    const ArmTypes ArmTypes::ECM("ECM", 6, ToolPose::Distance(1.0), {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});

}  // namespace saf

#endif  // DVRK_ARM_PARAMS_
