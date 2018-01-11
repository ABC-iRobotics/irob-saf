package irob_msgs;

public interface Environment extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/Environment";
  static final java.lang.String _DEFINITION = "int8 VALID = 1\nint8 INVALID = 2\n\n# Msg data\nHeader header\nint8 valid\n\nGraspObject[] objects\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte VALID = 1;
  static final byte INVALID = 2;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getValid();
  void setValid(byte value);
  java.util.List<irob_msgs.GraspObject> getObjects();
  void setObjects(java.util.List<irob_msgs.GraspObject> value);
}
