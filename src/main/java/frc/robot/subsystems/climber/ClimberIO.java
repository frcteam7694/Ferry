package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double encoder = 0.0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void drive(double power) {}

  default void zeroEncoder() {}
}
