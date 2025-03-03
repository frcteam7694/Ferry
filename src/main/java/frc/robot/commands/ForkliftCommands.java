package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.forklift.Forklift;

public class ForkliftCommands {

  public static Command drive(Forklift forklift, double power) {
    return new InstantCommand(() -> forklift.drive(power), forklift);
  }

  public static Command driveFor(Forklift forklift, double power, double time) {
    return drive(forklift, power).andThen(new WaitCommand(time)).andThen(drive(forklift, 0));
  }
}
