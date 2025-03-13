package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController pid;

  public Elevator(ElevatorIO io) {
    this.io = io;
    this.pid = new PIDController(kP, kI, kD);
    pid.setTolerance(kT);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/atSetpoint", atSetpoint());
  }

  public void goTo(double setpoint) {
    Logger.recordOutput("Elevator/setpoint", setpoint);
    pid.setSetpoint(setpoint);
  }

  // TODO: deprecate in favor of setpoints
  public void drive(double power) {
    io.drive(power);
  }

  public void setPointDrive() {
    drive(pid.calculate(inputs.encoder));
  }

  public double getEncoder() {
    return inputs.encoder;
  }

  public void zeroEncoder() {
    io.zeroEncoder(this);
  }

  public void buzzError(CommandXboxController controller) {
    controller.setRumble(GenericHID.RumbleType.kBothRumble, Math.abs(pid.getError()) / 382);
    io.resetAtBotton(this);
  }

  public boolean atSetpoint() {
    return pid.atSetpoint();
  }

  public void setTolerance(boolean start) {
    if (start) pid.setTolerance(DriverStation.isAutonomous() ? autoT : kT);
    else pid.setTolerance(kT);
  }
}
