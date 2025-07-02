package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class HorizontalExtension {
  private final CRServo rightExtensionServo;
  private final CRServo leftExtensionServo;

  private final double MAX_POWER = 1; // TODO: test + change
  private final double MIN_POWER = -1;

  private final Telemetry telemetry;
  private final HardwareMap hwMap;

  private long startTimeNanoSeconds = -1; //off


  public HorizontalExtension(HardwareMap hwMap, Telemetry telemetry) {
    this.hwMap = hwMap;
    this.telemetry = telemetry;

    rightExtensionServo = hwMap.crservo.get(Constants.HorizontalConstants.RIGHT_EXTENSION_ID);
    leftExtensionServo = hwMap.crservo.get(Constants.HorizontalConstants.LEFT_EXTENSION_ID);
    rightExtensionServo.setDirection(CRServo.Direction.REVERSE); // TODO: choose which servo
  }

  public void setPower(double rightPower, double leftPower){
    if(rightPower < MAX_POWER && rightPower > MIN_POWER &&
            leftPower < MAX_POWER && leftPower > MIN_POWER) {
      if(startTimeNanoSeconds == -1){
        leftExtensionServo.setPower(leftPower);
        startTimeNanoSeconds = System.nanoTime();
      }

      double elapsedTimeSec = (System.nanoTime() - startTimeNanoSeconds) / 1e9;
      if(elapsedTimeSec >= -.1){
        rightExtensionServo.setPower(rightPower);
      }
    }
  }

  public void extend() {
    setPower(Constants.HorizontalConstants.RIGHT_EXTEND_POWER, Constants.HorizontalConstants.LEFT_EXTEND_POWER);
  }

  public void retract() {
    setPower(Constants.HorizontalConstants.RIGHT_RETRACT_POWER, Constants.HorizontalConstants.LEFT_RETRACT_POWER);
  }

  public void stopServos(){
    rightExtensionServo.setPower(0);
    leftExtensionServo.setPower(0);
  }
}
