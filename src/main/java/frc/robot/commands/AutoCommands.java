package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.elevator.ElevateCommand;
import frc.robot.subsystems.dropper.Dropper;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class AutoCommands {

  public static Command dropL1(Elevator elevator, Dropper dropper) {
    return dropAtLevel(elevator, dropper, ElevatorConstants.level1);
  }

  public static Command dropL2(Elevator elevator, Dropper dropper) {
    return dropAtLevel(elevator, dropper, ElevatorConstants.level2);
  }

  public static Command dropL3(Elevator elevator, Dropper dropper) {
    return dropAtLevel(elevator, dropper, ElevatorConstants.level3);
  }

  public static Command dropL4(Elevator elevator, Dropper dropper) {
    return dropAtLevel(elevator, dropper, ElevatorConstants.level4);
  }

  private static Command dropAtLevel(Elevator elevator, Dropper dropper, int level) {
    return ElevateCommand.create(elevator, level)
        .andThen(DropperCommands.drop(dropper))
        .andThen(ElevateCommand.create(elevator, ElevatorConstants.level0));
  }

  public static Command goL0(Elevator elevator) {
    return goToLevel(elevator, ElevatorConstants.level0);
  }

  public static Command goL1(Elevator elevator) {
    return goToLevel(elevator, ElevatorConstants.level1);
  }

  public static Command goL2(Elevator elevator) {
    return goToLevel(elevator, ElevatorConstants.level2);
  }

  public static Command goL3(Elevator elevator) {
    return goToLevel(elevator, ElevatorConstants.level3);
  }

  public static Command goL4(Elevator elevator) {
    return goToLevel(elevator, ElevatorConstants.level4);
  }

  private static Command goToLevel(Elevator elevator, int level) {
    return ElevateCommand.create(elevator, level);
  }
}
