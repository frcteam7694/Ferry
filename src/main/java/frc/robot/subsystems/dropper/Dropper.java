package frc.robot.subsystems.dropper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Dropper extends SubsystemBase {

  private final DropperIO io;
  private final DropperIOInputsAutoLogged inputs = new DropperIOInputsAutoLogged();

  public Dropper(DropperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Dropper", inputs);
  }

  // TODO: deprecate in favor of setpoints
  public void drive(double power) {
    io.drive(power);
  }

  public double getEncoder() {
    return 0; // inputs.encoder;
  }
}
