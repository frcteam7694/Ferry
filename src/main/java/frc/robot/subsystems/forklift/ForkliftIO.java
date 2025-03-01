package frc.robot.subsystems.forklift;

import org.littletonrobotics.junction.AutoLog;

public interface ForkliftIO {
  @AutoLog
  class ForkliftIOInputs {
    public double encoder = 0.0;
  }

  default void updateInputs(ForkliftIOInputs inputs) {}

  default void drive(double power) {}

  default void zeroEncoder() {}
}
