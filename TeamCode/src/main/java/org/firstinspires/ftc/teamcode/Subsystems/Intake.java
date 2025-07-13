package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ClawIntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.ClawIntakePosition;

public class Intake {

  private ClawIntakePosition goalPositionClaw;
  private ClawIntakePosition goalPositionFlip;
  private final Servo clawIntakeServo;
  private final Servo intakeFlipServo;
  private final Servo clawRotationServo;

  private final Telemetry telemetry;

  public Intake(final HardwareMap hwMap, final Telemetry telemetry) {
    this.telemetry = telemetry;
    clawIntakeServo = hwMap.get(Servo.class, ClawIntakeConstants.CLAW_INTAKE_SERVO_ID);
    intakeFlipServo = hwMap.get(Servo.class, ClawIntakeConstants.INTAKE_FLIP_SERVO_ID);
    //intakeFlipServo.setDirection(Servo.Direction.REVERSE);
    clawRotationServo = hwMap.get(Servo.class, ClawIntakeConstants.CLAW_ROTATION_SERVO_ID);
  }

  public void goToPositionFlip(final ClawIntakePosition position) {
    if (position.position >= Servo.MIN_POSITION && position.position <= Servo.MAX_POSITION) {
      intakeFlipServo.setPosition(position.position);
    }
    goalPositionFlip = position;
  }

  public void goToPositionClaw(final ClawIntakePosition position) {
    if (position.position >= Servo.MIN_POSITION && position.position <= Servo.MAX_POSITION) {
      clawIntakeServo.setPosition(position.position);
    }
    goalPositionClaw = position;
  }

  public ClawIntakePosition[] getGoalPositions() {
    return new ClawIntakePosition[] {goalPositionClaw, goalPositionFlip};
  }
}
