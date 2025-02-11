package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double encoder = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void drive(double power) {}

    default void zeroEncoder() {}
}