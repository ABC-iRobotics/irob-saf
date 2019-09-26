package irob_msgs;

public interface Surgeme$Action extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_msgs/Surgeme$Action";
  static final java.lang.String _DEFINITION = "# Surgeme.action\n\n# Define the goal\n# Action types\nint8 STOP = 1\nint8 NAV_TO_POS = 2\nint8 GRASP = 3\nint8 CUT = 4\nint8 PUSH = 5\nint8 DISSECT = 6\nint8 PLACE = 7\nint8 MANIPULATE = 8\t\t# Manipulate grasped soft tissue\nint8 RELEASE = 9\nint8 MOVE_CAM = 10\n\n# Interpolation methods\nint8 INTERPOLATION_LINEAR = 1\nint8 INTERPOLATION_BEZIER = 2\n\n\nint8 action\n\n# Params\n# All params are in mm and deg\n\ngeometry_msgs/Pose\t target\t\t\t# GRASP, CUT, PLACE, PUSH, DISSECT, NAV_TO_POS\ngeometry_msgs/Pose[] waypoints\t\t# GRASP, CUT, PLACE, PUSH, DISSECT, NAV_TO_POS\nint8 \t\t\t\t interpolation\t# GRASP, CUT, PLACE, PUSH, DISSECT, NAV_TO_POS\ngeometry_msgs/Pose\t approach_pose\t# GRASP, CUT, PLACE, PUSH, DISSECT, RELEASE\nfloat64 \t\t \t target_diameter # GRASP, CUT, DISSECT,\n\t\t\t\t\t\t\t\t\t # PLACE, RELEASE, DISSECT\nfloat64 \t\t \t compression_rate # GRASP\ngeometry_msgs/Point  displacement\t# PUSH, DISSECT, MANIPULATE, MOVE_CAM\nfloat64\t\t\t\t speed_cartesian\n\t\t\t\t\t\t\t\t\t# NAV_TO_POS, GRASP, CUT, PUSH, PLACE, \n\t\t\t\t\t\t\t\t\t# DISSECT, MANIPULATE\nfloat64\t\t\t\t speed_jaw \t\t# GRASP, CUT, DISSECT, PUSH, RELEASE\n\n---\n\n# Define the result\nToolPose pose\nstring info\n\n---\n\n# Define a feedback message\nToolPose pose\nstring info\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = true;
}
