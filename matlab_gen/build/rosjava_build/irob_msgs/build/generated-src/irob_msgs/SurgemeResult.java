package irob_msgs;

public interface SurgemeResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/SurgemeResult";
  static final java.lang.String _DEFINITION = "\n# Define the result\nToolPose pose\nstring info\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  irob_msgs.ToolPose getPose();
  void setPose(irob_msgs.ToolPose value);
  java.lang.String getInfo();
  void setInfo(java.lang.String value);
}
