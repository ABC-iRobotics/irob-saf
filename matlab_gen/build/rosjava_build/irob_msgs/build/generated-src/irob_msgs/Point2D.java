package irob_msgs;

public interface Point2D extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/Point2D";
  static final java.lang.String _DEFINITION = "# Point2D.msg\n\nfloat64 x\nfloat64 y\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
}
