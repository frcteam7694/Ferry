package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.dropper.Dropper;

public class DropperCommands {

  public static Command drive(Dropper dropper, double power) {
    return new InstantCommand(() -> dropper.drive(power), dropper);
  }

  public static Command driveFor(Dropper dropper, double power, double time) {
    return drive(dropper, power).andThen(new WaitCommand(time)).andThen(drive(dropper, 0));
  }

  public static Command drop(Dropper dropper) {
    if (Robot.isSimulation()) return Commands.none();
    return driveFor(dropper, -1, .2).andThen(new WaitCommand(.5)).andThen(driveFor(dropper, 1, .1));
  }

  public static Command halfDrop(Dropper dropper) {
    return driveFor(dropper, -1, .2).andThen(new WaitCommand(.2)).andThen(driveFor(dropper, 1, .1));
  }
}
