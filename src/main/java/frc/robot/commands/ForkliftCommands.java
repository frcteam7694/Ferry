package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.forklift.Forklift;

public class ForkliftCommands {

  public static Command drive(Forklift forklift) {
    return Commands.run(forklift::setPointDrive, forklift);
  }

  public static Command setSetPoint(Forklift forklift, double setpoint) {
    return new InstantCommand(() -> forklift.goTo(setpoint), forklift);
  }
}
