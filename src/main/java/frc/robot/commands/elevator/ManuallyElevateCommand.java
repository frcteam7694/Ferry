package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ManuallyElevateCommand extends Command {

  Elevator elevator;
  DoubleSupplier power;
  Trigger end;

  public ManuallyElevateCommand(Elevator elevator, DoubleSupplier power, Trigger end) {
    this.elevator = elevator;
    this.power = power;
    this.end = end;
    addRequirements(elevator);
  }

  @Override
  public void execute() {
    elevator.drive(power.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.goTo(elevator.getEncoder());
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished() || !end.getAsBoolean();
  }
}
