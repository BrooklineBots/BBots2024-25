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
  private final DcMotor frontLeftMotor;
  private final DcMotor backLeftMotor;
  private final DcMotor frontRightMotor;
  private final DcMotor backRightMotor;
  private final Telemetry telemetry;
  private final IMU imu;
  private double fieldHeadingOffset = 0.0; // In radians
  private final HardwareMap hwMap;

  public MecanumDrive(final HardwareMap hardwareMap, final Telemetry telemetry) {
    this.hwMap = hardwareMap;
    this.telemetry = telemetry;

    // Declare our motors
    // Make sure your ID's match your configuration
    frontLeftMotor = hwMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
    backLeftMotor = hwMap.dcMotor.get(Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
    frontRightMotor = hwMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
    backRightMotor = hwMap.dcMotor.get(Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);

    // Reverse the right side motors. This may be wrong for your setup.
    // If your robot moves backwards when commanded to go forwards,
    // reverse the left side instead.
    // See the note about this earlier on this page.
    frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE); // TODO: UPDATE THESE
    backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    // Retrieve the IMU from the hardware map
    imu = hwMap.get(IMU.class, "imu"); // TODO: Check name
    // Adjust the orientation parameters to match your robot
    final IMU.Parameters parameters =
        new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // TODO: UPDATE THESE
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    imu.initialize(parameters);
  }

  public void setFieldHeadingOffset(final double offset) {
    fieldHeadingOffset = offset;
  }

  public void resetHeading() { // Dummy method
    resetYaw();
  }

  public void resetYaw() {
    // This button choice was made so that it is hard to hit on accident,
    // it can be freely changed based on preference.
    // The equivalent button is start on Xbox-style controllers.
    //      if (gamepad1.options) {
    //        imu.resetYaw();
    //      }
    if (imu != null) {
      imu.resetYaw();
    } else {
      telemetry.addData("IMU not initialized", "Cannot reset yaw");
    }
  }

  public void stop() {
    setPowers(0, 0, 0, 0);
  }

  public double getBotHeading() {
    if (imu != null) {
      // fieldHeadingOffset is normally 0.0
      return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + fieldHeadingOffset;
    } else {
      telemetry.addData("IMU not initialized", "Cannot get bot heading");
      return 0;
    }
  }

  /**
   * Drive the robot in a field-relative manner.
   *
   * @param y The forward/backward speed (-1 to 1)
   * @param x The left/right speed (-1 to 1)
   * @param rx The rotation speed (-1 to 1)
   */
  public void driveFieldRelative(final double y, final double x, final double rx) {
    //    Likely passed in values:
    //    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
    //    double x = gamepad1.left_stick_x;
    //    double rx = gamepad1.right_stick_x;

    final double botHeading = getBotHeading();
    // Rotate the movement direction counter to the bot's rotation
    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
    final double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

    rotX = rotX * 1.1; // Counteract imperfect strafing TODO: Update to driver preference

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio,
    // but only if at least one is out of the range [-1, 1]
    final double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
    final double frontLeftPower = (rotY + rotX + rx) / denominator;
    final double backLeftPower = (rotY - rotX + rx) / denominator;
    final double frontRightPower = (rotY - rotX - rx) / denominator;
    final double backRightPower = (rotY + rotX - rx) / denominator;

    frontLeftMotor.setPower(frontLeftPower);
    backLeftMotor.setPower(backLeftPower);
    frontRightMotor.setPower(frontRightPower);
    backRightMotor.setPower(backRightPower);
  }

  /**
   * Controls the motor powers specifically. Should only be used in testing or by the recorder
   * method.
   *
   * @param fLPower
   * @param fRPower
   * @param bLPower
   * @param bRPower
   */
  public void setPowers(
      final double fLPower, final double fRPower, final double bLPower, final double bRPower) {
    frontLeftMotor.setPower(fLPower);
    frontRightMotor.setPower(fRPower);
    backLeftMotor.setPower(bLPower);
    backRightMotor.setPower(bRPower);
  }

  /**
   * Get the current motor powers.
   *
   * @return An array of motor powers in the order: frontLeft, frontRight, backLeft, backRight
   */
  public double[] getMotorPowers() {
    return new double[] {
      frontLeftMotor.getPower(),
      frontRightMotor.getPower(),
      backLeftMotor.getPower(),
      backRightMotor.getPower()
    };
  }
}
