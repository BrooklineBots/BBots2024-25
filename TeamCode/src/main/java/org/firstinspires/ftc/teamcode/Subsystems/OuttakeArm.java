package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.*;

public class OuttakeArm {

  private OuttakeArmPosition goalPosition;
  private final Servo clawArm;

  private final Telemetry telemetry;

  public OuttakeArm(final HardwareMap hwMap, final Telemetry telemetry) {
    this.telemetry = telemetry;
    clawArm = hwMap.get(Servo.class, OuttakeArmConstants.CLAW_ARM_SERVO_ID);
    clawArm.setDirection(Servo.Direction.REVERSE);
    this.telemetry.addData("Direction:", clawArm.getDirection());
  }

  public void goToPosition(final OuttakeArmPosition position) {
    if (position.position >= Servo.MIN_POSITION && position.position <= Servo.MAX_POSITION) {
      clawArm.setPosition(position.position);
    }
    goalPosition = position;
  }

  public OuttakeArmPosition getGoalPosition() {
    return goalPosition;
  }
}
