package frc.robot.commands.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.maxDistancePerCommand;
import static frc.robot.subsystems.elevator.ElevatorConstants.midPoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.Elevator;

public class ElevateCommand extends Command {

  Elevator elevator;
  int level;

  private ElevateCommand(Elevator elevator, int level) {
    this.level = level;
    this.elevator = elevator;
    addRequirements(elevator);
    elevator.setTolerance(true);
  }

  public static Command create(Elevator elevator, int level) {
    if (Robot.isSimulation()) return Commands.none();
    if (Math.abs(elevator.getEncoder() - level) > maxDistancePerCommand) {
      int point = (level < elevator.getEncoder()) ? level - midPoint : midPoint;
      return new ParallelRaceGroup(new ElevateCommand(elevator, point), new WaitCommand(.15))
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
  public void end(boolean interrupted) {
    elevator.setTolerance(false);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || elevator.atSetpoint();
  }
}
