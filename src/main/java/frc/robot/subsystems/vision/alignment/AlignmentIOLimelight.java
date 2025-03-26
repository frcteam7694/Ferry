package frc.robot.subsystems.vision.alignment;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

/** IO implementation for real Limelight hardware. */
public class AlignmentIOLimelight implements AlignmentIO {

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber idSubscriber;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   */
  public AlignmentIOLimelight(String name) {
    var table = NetworkTableInstance.getDefault().getTable(name);
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    idSubscriber = table.getDoubleTopic("fID").subscribe(0.0);
  }

  @Override
  public void updateInputs(AlignmentIOInputs inputs) {
    // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

    // Update target observation
    inputs.latestTargetObservation =
        new TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

    // Save tag IDs to inputs objects
    int id = (int) idSubscriber.get();

    if (id == -1) inputs.tagIds = new int[0];
    else inputs.tagIds = new int[] {id};
  }
}
