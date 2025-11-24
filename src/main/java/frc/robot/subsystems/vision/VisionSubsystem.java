package frc.robot.subsystems.vision;

import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.estimator.PoseEstimator;
import frc.robot.RobotContainer.subsystems;
import frc.robot.constants.VisionConstants.CameraTransforms;
import lib.Helpers;

public class VisionSubsystem {
    private final List<Camera> m_cameras;

    public VisionSubsystem() {
        m_cameras = List.of(
            new Camera("frontleft", CameraTransforms.kFrontLeft),
            new Camera("frontright", CameraTransforms.kFrontRight),
            new Camera("backleft", CameraTransforms.kBackLeft),
            new Camera("backright", CameraTransforms.kBackRight)
        );
    }

    public List<AprilTag> getSeenAprilTags() {
        return m_cameras.stream()
            .flatMap(camera -> camera.getSeenAprilTags().stream())
            .collect(Collectors.toMap(
                tag -> tag.ID,
                tag -> tag,
                (existing, replacement) -> existing // keep first if ID duplicates
            ))
            .values().stream().toList();
    }

    public Optional<AprilTag> getClosestTag() {
        List<AprilTag> seenTags = getSeenAprilTags();

        if (seenTags.isEmpty()) return Optional.empty();

        return Optional.of(seenTags.stream().sorted(
            Comparator.comparingDouble(tag -> tag.pose.getTranslation().getDistance(
                Helpers.toTranslation3d(subsystems.driveSubsystem.getPose().getTranslation())
            ))
        ).toList().get(0));
    }
}
