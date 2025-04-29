package frc.robot.subsystems.forklift;

import static frc.robot.subsystems.forklift.ForkliftConstants.forkliftCanId;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.littletonrobotics.junction.Logger;

public class ForkliftIOVictorSPX implements ForkliftIO {

  private final WPI_VictorSPX victor;

  public ForkliftIOVictorSPX() {
    victor = new WPI_VictorSPX(forkliftCanId);
    victor.setNeutralMode(NeutralMode.Brake);
    victor.configVoltageCompSaturation(12);
  }

  @Override
  public void drive(double power) {
    Logger.recordOutput("Forklift/power", power);
    victor.set(power);
  }
}
