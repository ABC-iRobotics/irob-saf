package irob_msgs;

public interface GetControlVariablesRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/GetControlVariablesRequest";
  static final java.lang.String _DEFINITION = "#request \nfloat64[] input\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  double[] getInput();
  void setInput(double[] value);
}
