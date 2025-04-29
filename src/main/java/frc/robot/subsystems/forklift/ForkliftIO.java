package frc.robot.subsystems.forklift;

import org.littletonrobotics.junction.AutoLog;

public interface ForkliftIO {
  @AutoLog
  class ForkliftIOInputs {}

  default void updateInputs(ForkliftIOInputs inputs) {}

  default void drive(double power) {}
}
