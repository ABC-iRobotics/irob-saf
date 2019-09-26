package irob_msgs;

public interface InstrumentInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/InstrumentInfo";
  static final java.lang.String _DEFINITION = "# InstrumentInfo.msg\n# Instrument type\nint8 GRIPPER = 1\nint8 SCISSORS = 2\nint8 CAMERA = 3\n\n# Instrument information\nstring name\nint8 basic_type\nfloat64 jaw_length\t\t# in mm\nInstrumentJawPart[] jaw_parts\n\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte GRIPPER = 1;
  static final byte SCISSORS = 2;
  static final byte CAMERA = 3;
  java.lang.String getName();
  void setName(java.lang.String value);
  byte getBasicType();
  void setBasicType(byte value);
  double getJawLength();
  void setJawLength(double value);
  java.util.List<irob_msgs.InstrumentJawPart> getJawParts();
  void setJawParts(java.util.List<irob_msgs.InstrumentJawPart> value);
}
