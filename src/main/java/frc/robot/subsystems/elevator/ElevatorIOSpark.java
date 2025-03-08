package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {

  private final SparkMax leftSpark;
  private final SparkMax rightSpark;

  private final RelativeEncoder encoder;

  public ElevatorIOSpark() {
    leftSpark = new SparkMax(leftSparkCanId, SparkLowLevel.MotorType.kBrushless);
    rightSpark = new SparkMax(rightSparkCanId, SparkLowLevel.MotorType.kBrushless);
    encoder = rightSpark.getEncoder();
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0)
        .limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0)
        .limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);
    rightConfig.encoder.positionConversionFactor(2 * Math.PI).uvwMeasurementPeriod(10);
    tryUntilOk(
        leftSpark,
        5,
        () ->
            leftSpark.configure(
                leftConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
    tryUntilOk(
        rightSpark,
        5,
        () ->
            rightSpark.configure(
                rightConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.encoder = -encoder.getPosition();
  }

  @Override
  public void drive(double power) {
    Logger.recordOutput("Elevator/power", power);
    rightSpark.set(-power);
    leftSpark.set(power);
  }

  @Override
  public void zeroEncoder(Elevator elevator) {
    encoder.setPosition(0);
    elevator.goTo(0);
  }

  @Override
  public void resetAtBotton(Elevator elevator) {
    if (leftSpark.get() < 0 && leftSpark.getOutputCurrent() == 0 && elevator.getEncoder() < 8) {
      zeroEncoder(elevator);
    }
  }
}
