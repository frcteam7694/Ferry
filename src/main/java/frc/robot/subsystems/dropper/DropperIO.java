package frc.robot.subsystems.dropper;

import org.littletonrobotics.junction.AutoLog;

public interface DropperIO {
  @AutoLog
  class DropperIOInputs {
    public double encoder = 0.0;
  }

  default void updateInputs(DropperIOInputs inputs) {}

  default void drive(double power) {}

  default void zeroEncoder() {}
}
