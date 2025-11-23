package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Camera {
    private final PhotonCamera m_camera;
    private Optional<PhotonPipelineResult> m_latestResult;

    public Camera(String name) {
        m_camera = new PhotonCamera(name);
        m_latestResult = Optional.empty();
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        List<PhotonPipelineResult> latestFifoResults = m_camera.getAllUnreadResults();

        if (!latestFifoResults.isEmpty()) m_latestResult = Optional.of(latestFifoResults.get(0)); // get top of fifo if it exists

        return m_latestResult;
    }
}
