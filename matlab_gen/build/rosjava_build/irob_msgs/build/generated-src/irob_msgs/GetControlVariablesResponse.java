package irob_msgs;

public interface GetControlVariablesResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/GetControlVariablesResponse";
  static final java.lang.String _DEFINITION = "#response\nfloat64[] output";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double[] getOutput();
  void setOutput(double[] value);
}
