package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakePosition;

/** Claw subsystem that can open, close, flip, and auto-align using Limelight thresholding. */
public class LimelightClaw {

  private final Servo clawServo;
  private final Servo flipServo;
  private final CRServo rotateServo;
  private final Telemetry telemetry;
  private final Limelight limelight;

  // Tuning constants for vision alignment
  private static final double kP = 0.03;
  private static final double kMinPower = 0.05;
  private static final double kToleranceDeg = 1.0;

  public LimelightClaw(HardwareMap hwMap, Telemetry telemetry, Limelight limelight) {
    this.telemetry = telemetry;
    this.limelight = limelight;

    clawServo = hwMap.get(Servo.class, IntakeConstants.CLAW_INTAKE_SERVO_ID);
    flipServo = hwMap.get(Servo.class, IntakeConstants.INTAKE_FLIP_SERVO_ID);
    rotateServo = hwMap.get(CRServo.class, IntakeConstants.CLAW_ROTATION_SERVO_ID);
    rotateServo.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  /** Open the claw fully. */
  public void open() {
    clawServo.setPosition(IntakePosition.CLAW_OPEN_POSITION.position);
    telemetry.addLine("Claw: OPEN");
  }

  /** Close the claw fully. */
  public void close() {
    clawServo.setPosition(IntakePosition.CLAW_CLOSE_POSITION.position);
    telemetry.addLine("Claw: CLOSED");
  }

  /** Rotate claw with specified power. */
  public void rotate(double power) {
    rotateServo.setPower(power);
  }

  /** Stop rotating the claw. */
  public void stopRotate() {
    rotateServo.setPower(0);
  }

  /** Flip the claw to a preset angle. */
  public void flipTo(IntakePosition position) {
    flipServo.setPosition(position.position);
    telemetry.addData("Flip to", position.name());
  }

  /** Auto-align claw using Limelight’s tx (horizontal offset). */
  public void autoAlign() {
    if (!limelight.hasTarget()) {
      telemetry.addLine("Vision: no target");
      stopRotate();
      return;
    }

    double tx = limelight.getTx(); // Degrees off-center
    telemetry.addData("Vision tx", tx);

    if (Math.abs(tx) <= kToleranceDeg) {
      telemetry.addLine("Aligned!");
      stopRotate();
      return;
    }

    double power = kP * tx;
    if (Math.abs(power) < kMinPower) {
      power = Math.signum(power) * kMinPower;
    }

    power = Math.max(-1.0, Math.min(1.0, power));
    rotate(power);
    telemetry.addData("Rotate power", power);
  }
}
