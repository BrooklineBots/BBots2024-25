package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakePosition;

public class Intake {

  private IntakePosition goalPositionClaw;
  private IntakePosition goalPositionFlip;
  private final Servo clawIntakeServo;
  private final Servo intakeFlipServo;
  private final CRServo clawRotationServo;

  private final Telemetry telemetry;

  public Intake(final HardwareMap hwMap, final Telemetry telemetry) {
    this.telemetry = telemetry;
    clawIntakeServo = hwMap.get(Servo.class, IntakeConstants.CLAW_INTAKE_SERVO_ID);
    intakeFlipServo = hwMap.get(Servo.class, IntakeConstants.INTAKE_FLIP_SERVO_ID);
    // intakeFlipServo.setDirection(Servo.Direction.REVERSE);
    // clawIntakeServo.setDirection(Servo.Direction.REVERSE);
    clawRotationServo = hwMap.get(CRServo.class, IntakeConstants.CLAW_ROTATION_SERVO_ID);
    clawRotationServo.setDirection(DcMotorSimple.Direction.REVERSE);
  }

  public void goToPositionFlip(final IntakePosition position) {
    if (position.position >= Servo.MIN_POSITION && position.position <= Servo.MAX_POSITION) {
      intakeFlipServo.setPosition(position.position);
    }
    goalPositionFlip = position;
  }

  public void openClaw() {
    clawIntakeServo.setPosition(IntakePosition.CLAW_OPEN_POSITION.position);
  }

  public void closeClaw() {
    clawIntakeServo.setPosition(IntakePosition.CLAW_CLOSE_POSITION.position);
  }

  public void rotateIntake(double power) {
    clawRotationServo.setPower(power);
  }

  public void stopRotate() {
    clawRotationServo.setPower(0);
  }

  public IntakePosition[] getGoalPositions() {
    return new IntakePosition[] {goalPositionClaw, goalPositionFlip};
  }
}
