package irob_msgs;

public interface VisionObject extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/VisionObject";
  static final java.lang.String _DEFINITION = "# Material types\nint8 RIGID=1\nint8 SOFT=2\n\n# Msg data\nHeader header\n\n# Object info \nint8 id\nstring name\ngeometry_msgs/Pose pose\n\n# Grasping parameters\ngeometry_msgs/Pose grasp_pose\nfloat64 grasp_diameter\nint8 material\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte RIGID = 1;
  static final byte SOFT = 2;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getId();
  void setId(byte value);
  java.lang.String getName();
  void setName(java.lang.String value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  geometry_msgs.Pose getGraspPose();
  void setGraspPose(geometry_msgs.Pose value);
  double getGraspDiameter();
  void setGraspDiameter(double value);
  byte getMaterial();
  void setMaterial(byte value);
}
