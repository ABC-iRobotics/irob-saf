package irob_msgs;

public interface GraspObject extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/GraspObject";
  static final java.lang.String _DEFINITION = "# Object info \nint8 id\nstring name\ngeometry_msgs/Point position\n\n# Grasping parameters\ngeometry_msgs/Point grasp_position\ngeometry_msgs/Point approach_position\nfloat64 grasp_diameter\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  byte getId();
  void setId(byte value);
  java.lang.String getName();
  void setName(java.lang.String value);
  geometry_msgs.Point getPosition();
  void setPosition(geometry_msgs.Point value);
  geometry_msgs.Point getGraspPosition();
  void setGraspPosition(geometry_msgs.Point value);
  geometry_msgs.Point getApproachPosition();
  void setApproachPosition(geometry_msgs.Point value);
  double getGraspDiameter();
  void setGraspDiameter(double value);
}
