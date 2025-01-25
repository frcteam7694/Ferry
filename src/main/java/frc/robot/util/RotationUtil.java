package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationUtil {

  public static boolean within(Rotation2d a, Rotation2d b, Rotation2d tolerance) {
    return diff(a, b).getRadians() <= tolerance.getRadians();
  }

  public static Rotation2d diff(Rotation2d a, Rotation2d b) {
    return abs(a.rotateBy(b));
  }

  public static Rotation2d abs(Rotation2d in) {
    return in.getRadians() < 0 ? in.unaryMinus() : in;
  }
}
