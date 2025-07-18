package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

/**
 * Subsystem wrapper around the Limelight 3A camera.
 *
 * <p>Responsibilities:
 *
 * <ul>
 *   <li>Select the correct vision pipeline (alliance‑colour or yellow game piece).
 *   <li>Expose helper methods that return a rotation‐correction value that can be fed directly to
 *       {@link MecanumDrive#driveFieldRelative}.
 *   <li>Provide simple accessors such as {@code hasTarget()} and {@code getTx()} to allow
 *       higher‑level logic in {@code RobotContainer}.
 * </ul>
 */
public class Limelight {

  private final Limelight3A limelight;
  private final Telemetry telemetry;
  private Constants.LimelightConstants.Pipeline currentPipeline;

  public Limelight(
      final HardwareMap hwMap, final Telemetry telemetry, final boolean isRedAlliance) {
    this.telemetry = telemetry;
    this.limelight = hwMap.get(Limelight3A.class, "limelight");

    /* Default to alliance colour so drivers instantly get feedback. */
    setAlliance(isRedAlliance);
  }

  /* ------------------------------------------------------------ */
  /*                        CONFIGURATION                         */
  /* ------------------------------------------------------------ */

  /** Start image processing (call once during {@code init()} or {@code start()}). */
  public void start() {
    limelight.start();
  }

  /** Stop image processing to save power. */
  public void stop() {
    limelight.stop();
  }

  /**
   * Switches the Limelight to the correct pipeline for the current alliance.
   *
   * @param isRedAlliance
   */
  public void setAlliance(final boolean isRedAlliance) {
    switchPipeline(
        isRedAlliance
            ? Constants.LimelightConstants.Pipeline.RED
            : Constants.LimelightConstants.Pipeline.BLUE);
  }

  public boolean isRedAlliance() {
    return currentPipeline == Constants.LimelightConstants.Pipeline.RED;
  }

  public void setTargetYellow() {
    switchPipeline(Constants.LimelightConstants.Pipeline.YELLOW);
  }

  private void switchPipeline(final Constants.LimelightConstants.Pipeline newPipeline) {
    this.currentPipeline = newPipeline;
    limelight.pipelineSwitch(newPipeline.index);
  }

  public Constants.LimelightConstants.Pipeline getCurrentPipeline() {
    return currentPipeline;
  }

  /* ------------------------------------------------------------ */
  /*                          FEEDBACK                            */
  /* ------------------------------------------------------------ */

  /**
   * @return {@code true} when the Limelight currently sees a valid target.
   */
  public boolean hasTarget() {
    final LLResult res = limelight.getLatestResult();
    return res != null && res.isValid();
  }

  /** Horizontal offset (°) of the best target; {@code Double.NaN} if none. */
  public double getTx() {
    final LLResult res = limelight.getLatestResult();
    if (res != null && res.isValid()) {
      return res.getTx();
    }
    return Double.NaN;
  }

  /**
   * Calculates a rotation command in the range [-1, 1] that will drive the robot so the target (tx
   * ≈ 0) is centred. Returns 0 when no target is visible.
   */
  public double getRotationCorrection() {
    final double tx = getTx();
    if (Double.isNaN(tx)) {
      telemetry.addLine("Limelight: no target");
      return 0.0;
    }

    double cmd = Constants.LimelightConstants.kP * tx; // proportional steering
    // feed‑forward to overcome friction/stiction
    if (Math.abs(cmd) < Constants.LimelightConstants.kMinCmd)
      cmd = Math.signum(cmd) * Constants.LimelightConstants.kMinCmd;

    // clamp to valid motor range
    if (Math.abs(cmd) > 1.0) cmd = Math.signum(cmd);

    telemetry.addData("Limelight tx", tx);
    telemetry.addData("Rotate cmd", cmd);
    return cmd;
  }
}
