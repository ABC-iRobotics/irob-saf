package irob_msgs;

public interface SurgemeActionFeedback extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/SurgemeActionFeedback";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nirob_msgs/SurgemeFeedback feedback\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  irob_msgs.SurgemeFeedback getFeedback();
  void setFeedback(irob_msgs.SurgemeFeedback value);
}
