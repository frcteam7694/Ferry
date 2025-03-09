package frc.robot.commands.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.halfWayThrough;
import static frc.robot.subsystems.elevator.ElevatorConstants.maxDistancePerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;

public class ElevateCommand extends Command {

  Elevator elevator;
  int level;

  private ElevateCommand(Elevator elevator, int level) {
    this.level = level;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  public static Command create(Elevator elevator, int level) {
    if (Math.abs(elevator.getEncoder() - level) > maxDistancePerCommand) {
      return new ParallelRaceGroup(
              new ElevateCommand(elevator, halfWayThrough), new WaitCommand(.2))
          .andThen(new ElevateCommand(elevator, level));
    }
    return new ElevateCommand(elevator, level);
  }

  @Override
  public void initialize() {
    elevator.goTo(level);
    super.initialize();
  }

  @Override
  public void execute() {
    elevator.setPointDrive();
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || elevator.atSetpoint();
  }
}
