package irob_msgs;

public interface MarkerArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/MarkerArray";
  static final java.lang.String _DEFINITION = "# MarkerArray.msg\n\n# Msg data\nHeader header\nMarker[] markers\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<irob_msgs.Marker> getMarkers();
  void setMarkers(java.util.List<irob_msgs.Marker> value);
}
