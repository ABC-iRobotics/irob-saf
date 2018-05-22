package irob_msgs;

public interface TrajectoryToolPose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/TrajectoryToolPose";
  static final java.lang.String _DEFINITION = "# TrajectoryToolPose.msg\n# An array of poses and jaw angles with dt.\n\nfloat64 dt\n\nToolPose[] poses\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getDt();
  void setDt(double value);
  java.util.List<irob_msgs.ToolPose> getPoses();
  void setPoses(java.util.List<irob_msgs.ToolPose> value);
}
