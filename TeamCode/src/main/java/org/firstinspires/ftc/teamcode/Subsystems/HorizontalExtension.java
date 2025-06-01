package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class HorizontalExtension {
  private final Servo rightExtensionServo;
  // private final Servo leftExtensionServo;

  private final double MAX_POSITION = 1; // TODO: test + change
  private final double MIN_POSITION = 0;

  private final Telemetry telemetry;
  private final HardwareMap hwMap;

  public HorizontalExtension(HardwareMap hwMap, Telemetry telemetry) {
    this.hwMap = hwMap;
    this.telemetry = telemetry;

    rightExtensionServo = hwMap.servo.get(Constants.HorizontalConstants.RIGHT_EXTENSION_ID);
    // leftExtensionServo = hwMap.servo.get(Constants.HorizontalConstants.LEFT_EXTENSION_ID);
    rightExtensionServo.setDirection(Servo.Direction.REVERSE); // TODO: choose which servo

    telemetry.addData("Right Extension Position: ", rightExtensionServo.getPosition());
    // telemetry.addData("Left Extension Position: ", leftExtensionServo.getPosition());
  }

  public void setRightPosition(double position) {
    if (position >= MIN_POSITION && position <= MAX_POSITION) {
      rightExtensionServo.setPosition(position);
    }
  }

  // public void getLeftPosition(){leftExtensionServo.getPosition());}

  public double getRightPosition() {
    return rightExtensionServo.getPosition();
  }

  //  public void setLeftPosition(double position){
  //    if (position >= MIN_POSITION && position <= MAX_POSITION) {
  //      leftExtensionServo.setPosition(position);
  //    }
  //  }

  public void extendOut() {
    setRightPosition(Constants.HorizontalPosition.RIGHT_OUT_POSITION.position);
    // setLeftPosition(Constants.HorizontalPosition.LEFT_OUT_POSITION.position);
  }

  public void shrinkBack() {
    setRightPosition(Constants.HorizontalPosition.RIGHT_IN_POSITION.position);
    // setLeftPosition(Constants.HorizontalPosition.LEFT_IN_POSITION.position);
  }
}
