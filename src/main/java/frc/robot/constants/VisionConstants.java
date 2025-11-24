package frc.robot.constants;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    public static final class CameraTransforms {
      // Measure from the center line to the camera, not the 3D center of the robot.
      private static final Distance lengthWidth = Units.Inches.of(14);
      private static final Distance height = Units.Inches.of(8);
      private static final Angle pitch = Units.Degrees.of(-28.125);
      private static final Angle yawOffset = Units.Degrees.of(30);

      public static final Transform3d kFrontLeft = new Transform3d(
        -lengthWidth.in(Units.Meters),
        -lengthWidth.in(Units.Meters),
        height.in(Units.Meters), 
        new Rotation3d(0, pitch.in(Units.Radians), - Math.PI + yawOffset.in(Units.Radians))
      );
      public static final Transform3d kFrontRight = new Transform3d(
        -lengthWidth.in(Units.Meters),
        lengthWidth.in(Units.Meters),
        height.in(Units.Meters), 
        new Rotation3d(0, pitch.in(Units.Radians), Math.PI - yawOffset.in(Units.Radians))
      );
      public static final Transform3d kBackLeft = new Transform3d(
        lengthWidth.in(Units.Meters),
        -lengthWidth.in(Units.Meters),
        height.in(Units.Meters), 
        new Rotation3d(0, pitch.in(Units.Radians), - yawOffset.in(Units.Radians))
      );
      public static final Transform3d kBackRight = new Transform3d(
        lengthWidth.in(Units.Meters),
        lengthWidth.in(Units.Meters),
        height.in(Units.Meters), 
        new Rotation3d(0, pitch.in(Units.Radians), yawOffset.in(Units.Radians))
      );
    }

    public static final List<AprilTag> kTags = List.of( 
        new AprilTag(1, new Pose3d(2.8956, 0     , 1.7018, new Rotation3d(0, 0, Math.PI/2))),
        new AprilTag(2, new Pose3d(0     , 3.8202, 1.5494, new Rotation3d(0, 0, 0))),
        new AprilTag(3, new Pose3d(4.4260, 6.3754, 1.6129, new Rotation3d(0, 0, 3 * Math.PI / 2)))
    );

    public static final AprilTagFieldLayout kAprilTagLayout = new AprilTagFieldLayout(kTags, 10, 10);
}
