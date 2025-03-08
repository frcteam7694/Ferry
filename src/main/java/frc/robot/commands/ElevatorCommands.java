package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  public static Command drive(
      Elevator elevator,
      CommandXboxController controller,
      DoubleSupplier joystickPower,
      Trigger manual) {
    manual.onFalse(setSetPoint(elevator, elevator::getEncoder));
    return Commands.run(
        () -> {
          if (manual.getAsBoolean()) {
            elevator.drive(joystickPower.getAsDouble());
          } else {
            elevator.setPointDrive();
          }
          elevator.buzzError(controller);
        },
        elevator);
  }

  public static Command setSetPoint(Elevator elevator, double setpoint) {
    return new InstantCommand(() -> elevator.goTo(setpoint), elevator);
  }

  public static Command setSetPoint(Elevator elevator, DoubleSupplier setpointSupplier) {
    return new InstantCommand(() -> elevator.goTo(setpointSupplier.getAsDouble()), elevator);
  }

  public static Command resetEncoder(Elevator elevator) {
    return new InstantCommand(elevator::zeroEncoder).andThen(setSetPoint(elevator, 0));
  }
}
