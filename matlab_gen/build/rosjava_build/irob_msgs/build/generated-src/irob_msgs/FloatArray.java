package irob_msgs;

public interface FloatArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/FloatArray";
  static final java.lang.String _DEFINITION = "\nfloat64[] data\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double[] getData();
  void setData(double[] value);
}
