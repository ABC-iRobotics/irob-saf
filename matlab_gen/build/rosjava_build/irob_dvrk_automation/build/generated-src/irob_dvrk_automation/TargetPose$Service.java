package irob_dvrk_automation;

public interface TargetPose$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irob_dvrk_automation/TargetPose$Service";
  static final java.lang.String _DEFINITION = "#request constants\nint8 DISSECTION=1\nint8 DISTANT=2\n#request fields\nint8 target_type\n---\n#response constants\nint8 DP=3\nint8 GOAL=4\n#response fields\nint8 position_type\ngeometry_msgs/Pose pose\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
