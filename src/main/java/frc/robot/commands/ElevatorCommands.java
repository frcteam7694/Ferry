package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  public static Command joystickDrive(Elevator elevator, DoubleSupplier power) {
    return Commands.run(
        () -> {
          elevator.drive(power.getAsDouble());
        },
        elevator);
  }
}
