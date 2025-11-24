package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;

public class Camera {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;

    private Optional<PhotonPipelineResult> m_latestResult;

    public Camera(String name, Transform3d transformToCamera) {
        m_camera = new PhotonCamera(name);
        m_latestResult = Optional.empty();
        m_poseEstimator = new PhotonPoseEstimator(
            VisionConstants.kAprilTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            transformToCamera
        );

        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        List<PhotonPipelineResult> latestFifoResults = m_camera.getAllUnreadResults();

        if (!latestFifoResults.isEmpty()) m_latestResult = Optional.of(latestFifoResults.get(0)); // get top of fifo if it exists

        return m_latestResult;
    }

    public List<PhotonTrackedTarget> getSeenTargets() {
        getLatestResult(); // ensure result is fresh

        return m_latestResult.stream().flatMap(result -> result.getTargets().stream()).toList();
    }

    public List<AprilTag> getSeenAprilTags() {
        return getSeenTargets().stream().map(target -> target.getFiducialId()).map(
            id -> new AprilTag(
                id, 
                m_poseEstimator.getFieldTags().getTagPose(id).orElse(new Pose3d())
            )
        ).toList();
    }
}
