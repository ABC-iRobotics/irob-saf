package irob_dvrk_automation;

public interface TargetPoseRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_dvrk_automation/TargetPoseRequest";
  static final java.lang.String _DEFINITION = "#request constants\nint8 DISSECTION=1\nint8 DISTANT=2\n#request fields\nint8 target_type\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  static final byte DISSECTION = 1;
  static final byte DISTANT = 2;
  byte getTargetType();
  void setTargetType(byte value);
}
