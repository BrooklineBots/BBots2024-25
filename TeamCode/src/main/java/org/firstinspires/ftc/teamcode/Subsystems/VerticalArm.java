package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.*;

public class VerticalArm {

  private final DcMotor leftArm;
  private final DcMotor rightArm;
  private ArmPosition goalPosition;

  private final Telemetry telemetry;

  public VerticalArm(final HardwareMap hwMap, final Telemetry telemetry) {
    this.telemetry = telemetry;
    leftArm = hwMap.dcMotor.get(Constants.ArmConstants.LEFT_ARM_ID);
    rightArm = hwMap.dcMotor.get(Constants.ArmConstants.RIGHT_ARM_ID);
    

    //leftArm.setDirection(DcMotor.Direction.REVERSE);

    leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    resetEncoders();
    setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  private void setRunMode(final DcMotor.RunMode mode) {
    leftArm.setMode(mode);
    rightArm.setMode(mode);
  }

  private void resetEncoders() {
    setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  public void goToPosition(final ArmPosition position) {
    goalPosition = position;
    final int targetPosition = position.encoderTicks;

    if (targetPosition < ArmConstants.VERTICAL_MIN_POSITION
        || targetPosition > ArmConstants.VERTICAL_MAX_POSITION) {
      stop();
    }

    if (Math.abs(leftArm.getCurrentPosition() - rightArm.getCurrentPosition())
        > ArmConstants.MAX_ALLOWED_DIFFERENCE) {
      final double leftDistance = Math.abs(leftArm.getCurrentPosition() - targetPosition);
      final double rightDistance = Math.abs(rightArm.getCurrentPosition() - targetPosition);

      if (leftDistance >= rightDistance) {
        rightArm.setPower(0);
      } else {
        leftArm.setPower(0);
      }
    }

    leftArm.setTargetPosition(targetPosition);
    rightArm.setTargetPosition(targetPosition);

    setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftArm.setPower(ArmConstants.VERTICAL_MOVE_POWER);
    rightArm.setPower(ArmConstants.VERTICAL_MOVE_POWER);
  }

  public ArmPosition getGoalPosition() {
    return goalPosition;
  }

  public double[] getCurrentPosition() {
    return new double[] {leftArm.getCurrentPosition(), rightArm.getCurrentPosition()};
  }

  public double[] getArmPowers() {
    return new double[] {leftArm.getPower(), rightArm.getPower()};
  }

  public void setModeRunWithoutEncoder() {
    leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  public void setModeRunWithEncoder() {
    leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }

  public void setArmPowers(final double power) {
    leftArm.setPower(power);
    rightArm.setPower(power);
  }

  public void stop() {
    leftArm.setPower(0);
    rightArm.setPower(0);
  }
}
