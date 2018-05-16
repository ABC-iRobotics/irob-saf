package irob_msgs;

public interface ToolPoseStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/ToolPoseStamped";
  static final java.lang.String _DEFINITION = "# ToolPoseStamped.msg\n# A representation of robot tool pose in free space, composed of position,\n# orientation and jaw angle with a header.\n\nHeader header\n \nToolPose pose\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  irob_msgs.ToolPose getPose();
  void setPose(irob_msgs.ToolPose value);
}
