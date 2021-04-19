package irob_msgs;

public interface Robot$Action extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/Robot$Action";
  static final java.lang.String _DEFINITION = "# Robot.action\n\n# Define the goal\n# Action types\nint8 STOP=1\nint8 INIT_ARM=2\nint8 RESET_POSE=3\nint8 FOLLOW_TRAJECTORY=4\nint8 MOVE_JOINT=5\n\nint8 action\n\n# Params\nbool move_allowed\nTrajectoryToolPose trajectory\nsensor_msgs/JointState joint_state\n\n---\n# Define the result\nToolPose pose\nstring info\n---\n# Define a feedback message\nToolPose pose\nstring info\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
}
