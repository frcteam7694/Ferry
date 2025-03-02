package frc.robot.subsystems.forklift;

import static frc.robot.subsystems.forklift.ForkliftConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Forklift extends SubsystemBase {

  private final ForkliftIO io;
  private final ForkliftIOInputsAutoLogged inputs = new ForkliftIOInputsAutoLogged();
  private final PIDController pid;

  public Forklift(ForkliftIO io) {
    this.io = io;
    this.pid = new PIDController(kP, kI, kD);
    pid.enableContinuousInput(0, 1);
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

  public void goTo(double setpoint) {
    Logger.recordOutput("Elevator/setpoint", setpoint);
    pid.setSetpoint(setpoint);
  }

  public void setPointDrive() {
    drive(pid.calculate(inputs.encoder));
  }

  public void zeroEncoder() {
    io.zeroEncoder();
  }
}
