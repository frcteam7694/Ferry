package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationUtil {

  public static boolean within(Rotation2d a, Rotation2d b, Rotation2d tolerance) {
    return diff(a, b).getRadians() <= tolerance.getRadians();
  }

  public static Rotation2d diff(Rotation2d a, Rotation2d b) {
    return normalize(abs(a.rotateBy(b.unaryMinus())));
  }

  public static Rotation2d abs(Rotation2d in) {
    return in.getRadians() < 0 ? in.unaryMinus() : in;
  }

  public static Rotation2d normalize(Rotation2d in) {
    return (in.getRadians() <= Math.PI) ? in : new Rotation2d(2 * Math.PI - in.getRadians());
  }

}
