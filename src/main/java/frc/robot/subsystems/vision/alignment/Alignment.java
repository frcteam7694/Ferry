package frc.robot.subsystems.vision.alignment;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Alignment extends SubsystemBase {

  private final Supplier<Rotation2d> gyro;
  private final AlignmentIO[] io;
  private final AlignmentIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Alignment(Supplier<Rotation2d> gyro, AlignmentIO... io) {
    this.gyro = gyro;
    this.io = io;

    // Initialize inputs
    this.inputs = new AlignmentIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AlignmentIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Alignment camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with tracking.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {}
}
