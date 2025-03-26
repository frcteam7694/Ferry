package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommands {

  public static Command drive(Climber climber, Trigger unlock, CommandXboxController power) {
    return new RunCommand(
        () -> {
          if (unlock.getAsBoolean()) {
            climber.drive(power.getLeftY());
          } else {
            climber.drive(0);
          }
        },
        climber);
  }

  public static Command enableCamera(Climber climber) {
    return new InstantCommand(climber::enableCamera, climber);
  }
}
