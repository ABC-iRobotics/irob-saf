package irob_msgs;

public interface Marker extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/Marker";
  static final java.lang.String _DEFINITION = "# Marker.msg\n\n# Msg data\nint8 id\nPoint2D[] corners\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  byte getId();
  void setId(byte value);
  java.util.List<irob_msgs.Point2D> getCorners();
  void setCorners(java.util.List<irob_msgs.Point2D> value);
}
