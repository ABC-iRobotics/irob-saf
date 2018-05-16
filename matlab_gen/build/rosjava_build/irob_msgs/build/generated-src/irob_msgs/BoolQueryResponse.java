package irob_msgs;

public interface BoolQueryResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/BoolQueryResponse";
  static final java.lang.String _DEFINITION = "#response fields\nbool data";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getData();
  void setData(boolean value);
}
