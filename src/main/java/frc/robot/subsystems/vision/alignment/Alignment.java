package frc.robot.subsystems.vision.alignment;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Alignment extends SubsystemBase {

  private final Supplier<Rotation2d> gyro;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Alignment(Supplier<Rotation2d> gyro, VisionIO... io) {
    this.gyro = gyro;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert("Alignment camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the best target, which can be used for simple servoing with tracking.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public VisionIO.TargetObservation getTarget(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation;
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Alignment/Camera" + i, inputs[i]);
    }
  }
}
