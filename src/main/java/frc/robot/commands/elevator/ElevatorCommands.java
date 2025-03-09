package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {

  public static Command drive(Elevator elevator, CommandXboxController controller) {
    return Commands.run(
        () -> {
          elevator.setPointDrive();
          elevator.buzzError(controller);
        },
        elevator);
  }

  public static Command resetEncoder(Elevator elevator) {
    return new InstantCommand(elevator::zeroEncoder);
  }
}
