package frc.robot.subsystems.climber;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private boolean runningCamera = false;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  // TODO: deprecate in favor of setpoints
  public void drive(double power) {
    io.drive(power);
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }

  public void enableCamera() {
    if (runningCamera) return;
    if (Robot.isReal()) {
      var camera1 = CameraServer.startAutomaticCapture();
      camera1.setResolution(212, 160);
      camera1.setFPS(25);
    }
    runningCamera = true;
  }
}
