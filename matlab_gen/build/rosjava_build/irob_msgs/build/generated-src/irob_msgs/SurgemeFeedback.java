package irob_msgs;

public interface SurgemeFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/SurgemeFeedback";
  static final java.lang.String _DEFINITION = "\n# Define a feedback message\nToolPose pose\nstring info";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  irob_msgs.ToolPose getPose();
  void setPose(irob_msgs.ToolPose value);
  java.lang.String getInfo();
  void setInfo(java.lang.String value);
}
