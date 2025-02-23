package frc.robot.subsystems.dropper;

import static frc.robot.subsystems.dropper.DropperConstants.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.littletonrobotics.junction.Logger;

public class DropperIOSpark implements DropperIO {

  private final SparkMax spark;

  private final AbsoluteEncoder encoder;

  public DropperIOSpark() {
    spark = new SparkMax(dropperCanId, SparkLowLevel.MotorType.kBrushed);
    encoder = spark.getAbsoluteEncoder();
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(SparkBaseConfig.IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(currentLimit)
        .voltageCompensation(12.0)
        .limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);
    config.encoder.positionConversionFactor(2 * Math.PI).uvwMeasurementPeriod(10);
    tryUntilOk(
        spark,
        5,
        () ->
            spark.configure(
                config,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(DropperIOInputs inputs) {
    inputs.encoder = encoder.getPosition();
  }

  @Override
  public void drive(double power) {
    Logger.recordOutput("Dropper/power", power);
    spark.set(power);
  }
}
