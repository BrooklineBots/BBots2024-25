package org.firstinspires.ftc.teamcode;

public class Utils {
  public static double clamp(final double val, final double min, final double max) {
    return Math.max(min, Math.min(max, val));
  }

  public boolean isWithinTolerance(
      final double targetValue, final double currentValue, final double tolerance) {
    return Math.abs(targetValue - currentValue) <= tolerance;
  }

  public static double ticksToInches(
      final double ticks, final double wheelDiameter, final double gearRatio) {
    final double TICKS_PER_REV = 384.5; // specific to GoBILDA 13.7:1 Yellow Jacket
    return (ticks / (gearRatio * TICKS_PER_REV)) * (Math.PI * wheelDiameter);
  }
}
