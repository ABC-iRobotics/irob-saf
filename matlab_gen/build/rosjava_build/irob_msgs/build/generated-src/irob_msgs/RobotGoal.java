package irob_msgs;

public interface RobotGoal extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/RobotGoal";
  static final java.lang.String _DEFINITION = "# Define the goal\n# Action types\nint8 STOP=1\nint8 INIT_ARM=2\nint8 RESET_POSE=3\nint8 FOLLOW_TRAJECTORY=4\n\nint8 action\n\n# Params\nbool move_allowed\nTrajectoryToolPose trajectory\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  static final byte STOP = 1;
  static final byte INIT_ARM = 2;
  static final byte RESET_POSE = 3;
  static final byte FOLLOW_TRAJECTORY = 4;
  byte getAction();
  void setAction(byte value);
  boolean getMoveAllowed();
  void setMoveAllowed(boolean value);
  irob_msgs.TrajectoryToolPose getTrajectory();
  void setTrajectory(irob_msgs.TrajectoryToolPose value);
}
