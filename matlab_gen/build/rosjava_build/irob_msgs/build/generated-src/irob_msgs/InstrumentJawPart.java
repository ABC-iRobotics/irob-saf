package irob_msgs;

public interface InstrumentJawPart extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/InstrumentJawPart";
  static final java.lang.String _DEFINITION = "# InstrumentJawPart.msg\n# Jaw part type\nint8 JOINT = 1\nint8 GRIPPER = 2\nint8 SCISSORS = 3\n\n# Jaw part information\nint8 type\nfloat64 start\t\t# on jaw length from TCP in mm\nfloat64 end\t\t\t# on jaw length from TCP in mm\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte JOINT = 1;
  static final byte GRIPPER = 2;
  static final byte SCISSORS = 3;
  byte getType();
  void setType(byte value);
  double getStart();
  void setStart(double value);
  double getEnd();
  void setEnd(double value);
}
