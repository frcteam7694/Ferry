package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  public static Command setPointDrive(Elevator elevator) {
    return Commands.run(elevator::setPointDrive, elevator);
  }

  public static Command setSetPoint(Elevator elevator, double setpoint) {
    return new InstantCommand(() -> elevator.goTo(setpoint));
  }

  public static Command joystickDrive(Elevator elevator, DoubleSupplier power) {
    return Commands.run(() -> elevator.drive(power.getAsDouble()), elevator);
  }
}
