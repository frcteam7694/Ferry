package frc.robot.subsystems.forklift;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Forklift extends SubsystemBase {

  private final ForkliftIO io;
  private final ForkliftIOInputsAutoLogged inputs = new ForkliftIOInputsAutoLogged();

  public Forklift(ForkliftIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Forklift", inputs);
  }

  // TODO: deprecate in favor of setpoints
  public void drive(double power) {
    io.drive(power);
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }
}
