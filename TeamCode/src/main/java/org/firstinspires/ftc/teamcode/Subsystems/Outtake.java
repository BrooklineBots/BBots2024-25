package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.*;

public class Outtake {

  private final double MAX_POSITION = 1;
  private final double MIN_POSITION = 0;
  private OuttakePosition goalPosition;
  private final Servo outtake;

  private final Telemetry telemetry;

  public Outtake(final HardwareMap hwMap, final Telemetry telemetry) {
    this.telemetry = telemetry;
    outtake = hwMap.get(Servo.class, OuttakeConstants.CLAW_SERVO_ID);
    outtake.setDirection(Servo.Direction.REVERSE);
    goalPosition = OuttakePosition.OPEN_POSITION;
    telemetry.addData("Position:", outtake.getPosition());

  }

  public void openClaw() {
    setPosition(OuttakePosition.OPEN_POSITION.position);
    goalPosition = OuttakePosition.OPEN_POSITION;
  }

  public void closeClaw() {
    setPosition(OuttakePosition.CLOSE_POSITION.position);
    goalPosition = OuttakePosition.CLOSE_POSITION;
  }

  public OuttakePosition getGoalPosition() {
    return goalPosition;
  }

  public void setPosition(final double position) {
    if (position >= MIN_POSITION && position <= MAX_POSITION) {
      outtake.setPosition(position);
    }
  }

  public double getClawPosition() {
    return outtake.getPosition();
  }
}
