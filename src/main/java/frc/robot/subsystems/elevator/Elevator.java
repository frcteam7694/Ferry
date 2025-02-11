package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final PIDController pid;

    public Elevator(ElevatorIO io) {
        this.io = io;
        this.pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    public void goTo(double setpoint) {
        pid.setSetpoint(setpoint);
    }

    // TODO: deprecate in favor of setpoints
    public void drive(double power) {
        io.drive(power);
    }

    public void setPointDrive() {
        drive(pid.calculate(inputs.encoder));
    }

    public double getEncoder() {
        return inputs.encoder;
    }

    public void zeroEncoder() {
        io.zeroEncoder();
    }
}