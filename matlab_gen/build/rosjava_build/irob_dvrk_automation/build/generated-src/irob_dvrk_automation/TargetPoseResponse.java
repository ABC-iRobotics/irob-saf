package irob_dvrk_automation;

public interface TargetPoseResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_dvrk_automation/TargetPoseResponse";
  static final java.lang.String _DEFINITION = "#response constants\nint8 DP=3\nint8 GOAL=4\n#response fields\nint8 position_type\ngeometry_msgs/Pose pose";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte DP = 3;
  static final byte GOAL = 4;
  byte getPositionType();
  void setPositionType(byte value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
}
