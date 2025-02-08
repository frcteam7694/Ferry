package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.Elastic;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {

  public static Command drive(Elevator elevator, DoubleSupplier joystickPower, Trigger manual) {
    manual.onFalse(setSetPoint(elevator, elevator::getEncoder));
    manual.onFalse(
        new InstantCommand(
            () ->
                Elastic.sendNotification(
                    new Elastic.Notification(
                        Elastic.Notification.NotificationLevel.INFO, "", "done did"))));
    return Commands.run(
        () -> {
          if (manual.getAsBoolean()) {
            elevator.drive(joystickPower.getAsDouble());
          } else {
            elevator.setPointDrive();
          }
        },
        elevator);
  }

  public static Command setSetPoint(Elevator elevator, double setpoint) {
    return new InstantCommand(() -> elevator.goTo(setpoint), elevator);
  }

  public static Command setSetPoint(Elevator elevator, DoubleSupplier setpointSupplier) {
    return new InstantCommand(() -> elevator.goTo(setpointSupplier.getAsDouble()), elevator);
  }
}
