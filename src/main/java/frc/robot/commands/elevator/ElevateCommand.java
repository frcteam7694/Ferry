package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevateCommand extends Command {

  Elevator elevator;
  int level;

  public ElevateCommand(Elevator elevator, int level) {
    this.level = level;
    this.elevator = elevator;
  }

  public ElevateCommand(Elevator elevator, DoubleSupplier level) {
    this(elevator, (int) level.getAsDouble());
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
