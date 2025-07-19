package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive {
  // Declare our motors
  private final DcMotor frontLeftMotor;
  private final DcMotor backLeftMotor;
  private final DcMotor frontRightMotor;
  private final DcMotor backRightMotor;

  private final IMU imu;

  private final Telemetry telemetry;

  private double fieldHeadingOffset = 0.0; // in radians

  private final HardwareMap hwMap;

  // Make sure your ID's match your configuration
  public MecanumDrive(final HardwareMap hwMap, final Telemetry telemetry) {
    this.hwMap = hwMap;
    this.telemetry = telemetry;

    frontLeftMotor = hwMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
    backLeftMotor = hwMap.dcMotor.get(Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
    frontRightMotor = hwMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
    backRightMotor = hwMap.dcMotor.get(Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);

    frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    //backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


    frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Retrieve the IMU from the hardware map
    imu = hwMap.get(IMU.class, Constants.DriveConstants.IMU_ID);
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters =
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    imu.initialize(parameters);
  }

  /**
   * Drives robot in a field relative manner
   *
   * @param y = forward/backward speed, -1 to 1
   * @param x = left/right speed, -1 to 1
   * @param rx = rotation speed, -1 to 1
   */
  public void driveFieldRelative(double y, double x, double rx) {

    // Rotate the movement direction counter to the bot's rotation
    double heading = getBotHeading();
    double rotX = x * Math.cos(heading) - y * Math.sin(heading);
    double rotY = x * Math.sin(heading) + y * Math.cos(heading);

    double rotXa = rotX * 1.11;// Counteract imperfect strafing

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    double frontLeftPower = (rotY + rotXa + rx) / denominator;
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    double backRightPower = (rotY + rotXa - rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);
  }

  public void resetYaw() {
    imu.resetYaw();
  }

  public double[] getPowers() {
    return new double[] {
      frontLeftMotor.getPower(),
      frontRightMotor.getPower(),
      backLeftMotor.getPower(),
      backRightMotor.getPower()
    };
  }

  public double getBotHeading() {
    double botHeading =
            -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) + fieldHeadingOffset;
    return botHeading;
  }

  public void setFieldHeadingOffset(final double newOffset) {
    fieldHeadingOffset = newOffset;
  }

  /**
   * Sets exact motor powers
   *
   * @param fLPower = frontLeft speed, -1 to 1
   * @param fRPower = frontRight speed, -1 to 1
   * @param bLPower = backLeft speed, -1 to 1
   * @param bRPower = backRight speed, -1 to 1
   */
  public void setPowers(double fLPower, double fRPower, double bLPower, double bRPower) {

    double maxSpeed = 1.0;
    maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
    maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));
    maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
    maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));

    fLPower /= maxSpeed;
    fRPower /= maxSpeed;
    bLPower /= maxSpeed;
    bRPower /= maxSpeed;

    frontLeftMotor.setPower(fLPower);
    frontRightMotor.setPower(fRPower);
    backLeftMotor.setPower(bLPower);
    backRightMotor.setPower(bRPower);
  }

  public void driveRobotCentric(double forward, double right, double rotate){
    double fLPower = forward + right + rotate;
    double fRPower = forward - right - rotate;
    double bLPower = forward - right + rotate;
    double bRPower = forward + right - rotate;

    setPowers(fLPower, fRPower, bLPower, bRPower);
  }

  public void stopMotors() {
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    backRightMotor.setPower(0);
    backLeftMotor.setPower(0);
  }
}
