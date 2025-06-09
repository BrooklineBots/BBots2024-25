package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class HorizontalExtension {
  private final CRServo rightExtensionServo;
  // private final Servo leftExtensionServo;

  private final double MAX_POSITION = 1; // TODO: test + change
  private final double MIN_POSITION = 0;

  private final Telemetry telemetry;
  private final HardwareMap hwMap;

  public HorizontalExtension(HardwareMap hwMap, Telemetry telemetry) {
    this.hwMap = hwMap;
    this.telemetry = telemetry;

    rightExtensionServo = hwMap.crservo.get(Constants.HorizontalConstants.RIGHT_EXTENSION_ID);
    // leftExtensionServo = hwMap.servo.get(Constants.HorizontalConstants.LEFT_EXTENSION_ID);
    rightExtensionServo.setDirection(CRServo.Direction.REVERSE); // TODO: choose which servo

    // telemetry.addData("Left Extension Position: ", leftExtensionServo.getPosition());
  }


  public void extendOut() {
    rightExtensionServo.setPower(1);
  }

  public void shrinkBack() {
    rightExtensionServo.setPower(-1);
  }

  public void stopServos(){
    rightExtensionServo.setPower(0);
  }
}
