package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int leftSparkCanId = 10;
  public static final int rightSparkCanId = 11;

  public static final int currentLimit = 50;

  public static final double kP = .005;
  public static final double kI = .001;
  public static final double kD = 0;

  public static int level0 = 0;
  public static int level1 = 95;
  public static int level2 = 120;
  public static int level3 = 215;
  public static int level4 = 393;

  public static int maxDistancePerCommand = 200;
  public static int halfWayThrough = level4 / 2;
}
