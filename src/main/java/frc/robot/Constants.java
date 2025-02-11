package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final Robots currentRobot =
      BuildConstants.DEPLOY_SSID.contains("FRC") ? Robots.Ferry : Robots.Terry;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public enum Robots {
    Terry,
    Ferry;
  }
}
