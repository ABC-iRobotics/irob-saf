package irob_msgs;

public interface ToolPose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/ToolPose";
  static final java.lang.String _DEFINITION = "# A representation of robot tool pose in free space, composed of position,\n# orientation and jaw angle.\n \ngeometry_msgs/Point position\ngeometry_msgs/Quaternion orientation\nfloat64 jaw\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
  double getJaw();
  void setJaw(double value);
}
