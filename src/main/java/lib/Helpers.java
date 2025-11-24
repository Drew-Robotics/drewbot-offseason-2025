package lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;

public class Helpers {
    public static Pose3d toPose3d(Pose2d pose) {
        return new Pose3d(
            pose.getMeasureX(),
            Units.Meters.zero(),
            pose.getMeasureY(),
            new Rotation3d(pose.getRotation())
        );
    }

    public static Translation3d toTranslation3d(Translation2d translation) {
        return new Translation3d(
            translation.getMeasureX(),
            Units.Meters.zero(),
            translation.getMeasureY()
        );
    }
}
