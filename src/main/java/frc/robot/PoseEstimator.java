package frc.robot;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseEstimator {
    private final Supplier<Pose2d> m_kinematicsPoseSupplier;
    private final Supplier<List<Optional<EstimatedRobotPose>>> m_cameraPosesSupplier;

    public PoseEstimator(Supplier<Pose2d> kinematicsPoseSupplier, Supplier<List<Optional<EstimatedRobotPose>>> cameraPosesSupplier) {
        m_kinematicsPoseSupplier = kinematicsPoseSupplier;
        m_cameraPosesSupplier = cameraPosesSupplier;
    }
}
