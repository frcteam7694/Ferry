package frc.robot.subsystems.dropper;

import org.littletonrobotics.junction.AutoLog;

public interface DropperIO {
  @AutoLog
  class DropperIOInputs {}

  default void updateInputs(DropperIOInputs inputs) {}

  default void drive(double power) {}
}
