package irob_msgs;

public interface RobotGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/RobotGoal";
  static final java.lang.String _DEFINITION = "# Robot.action\n\n# Define the goal\n# Action types\nint8 STOP=1\nint8 INIT_ARM=2\nint8 RESET_POSE=3\nint8 FOLLOW_TRAJECTORY=4\nint8 MOVE_JOINT=5\n\nint8 action\n\n# Params\nbool move_allowed\nTrajectoryToolPose trajectory\nsensor_msgs/JointState joint_state\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  static final byte STOP = 1;
  static final byte INIT_ARM = 2;
  static final byte RESET_POSE = 3;
  static final byte FOLLOW_TRAJECTORY = 4;
  static final byte MOVE_JOINT = 5;
  byte getAction();
  void setAction(byte value);
  boolean getMoveAllowed();
  void setMoveAllowed(boolean value);
  irob_msgs.TrajectoryToolPose getTrajectory();
  void setTrajectory(irob_msgs.TrajectoryToolPose value);
  sensor_msgs.JointState getJointState();
  void setJointState(sensor_msgs.JointState value);
}
