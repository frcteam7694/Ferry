package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int leftSparkCanId = 10;
  public static final int rightSparkCanId = 11;

  public static final int currentLimit = 50;

  public static final double kP = .01;
  public static final double kI = .002;
  public static final double kD = 0;
  public static final double kT = 2;
  public static final double autoT = 10;

  public static int level0 = 0;
  public static int level1 = 95;
  public static int level2 = 120;
  public static int level3 = 215;
  public static int level4 = 383;

  public static int maxDistancePerCommand = 250;
  public static int midPoint = level4 / 5;
}
