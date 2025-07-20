package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Limelight {

  private final Limelight3A limelight;
  private final Telemetry telemetry;
  private Constants.LimelightConstants.Pipeline currentPipeline;

  public Limelight(HardwareMap hwMap, Telemetry telemetry, boolean isRedAlliance) {
    this.telemetry = telemetry;
    this.limelight = hwMap.get(Limelight3A.class, "limelight");
    setAlliance(isRedAlliance);
  }

  public void start() {
    limelight.start();
  }

  public void stop() {
    limelight.stop();
  }

  public void setAlliance(boolean isRedAlliance) {
    switchPipeline(isRedAlliance
            ? Constants.LimelightConstants.Pipeline.RED
            : Constants.LimelightConstants.Pipeline.BLUE);
  }

  public void setTargetYellow() {
    switchPipeline(Constants.LimelightConstants.Pipeline.YELLOW);
  }

  private void switchPipeline(Constants.LimelightConstants.Pipeline newPipeline) {
    this.currentPipeline = newPipeline;
    limelight.pipelineSwitch(newPipeline.index);
  }

  public boolean hasTarget() {
    final LLResult res = limelight.getLatestResult();
    return res != null && res.isValid();
  }

  public double getTx() {
    final LLResult res = limelight.getLatestResult();
    if (res != null && res.isValid()) {
      return res.getTx();
    }
    return Double.NaN;
  }

  public double getClawAlignmentPower() {
    final double tx = getTx();
    if (Double.isNaN(tx)) {
      telemetry.addLine("Limelight: no target");
      return 0.0;
    }

    double cmd = Constants.LimelightConstants.kP * tx;

    if (Math.abs(cmd) < Constants.LimelightConstants.kMinCmd)
      cmd = Math.signum(cmd) * Constants.LimelightConstants.kMinCmd;

    cmd = Math.max(-1.0, Math.min(1.0, cmd));

    telemetry.addData("tx", tx);
    telemetry.addData("Claw Align Power", cmd);
    return cmd;
  }
}
