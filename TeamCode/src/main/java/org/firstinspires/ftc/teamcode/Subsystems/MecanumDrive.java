package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive {
  private final DcMotor frontLeftMotor;
  private final DcMotor frontRightMotor;
  private final DcMotor backLeftMotor;
  private final DcMotor backRightMotor;
  private Telemetry telemetry;

  public MecanumDrive(final HardwareMap hardwareMap, final Telemetry telemetry) {
    frontLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
    frontRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
    backLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
    backRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);
    this.telemetry = telemetry;

    frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

    // Generate a random number between 10 and 100 (inclusive)

    //        telemetry.addData("IMU Angle:",
    // imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
    //        telemetry.update();
    this.telemetry = telemetry;
  }

  private void setPowers(
      final double fLPower, final double fRPower, final double bLPower, final double bRPower) {
    // finds the highest speed b/w 1 and fL then that and fR and so on
    // double maxSpeed = Math.max(Math.abs(fRPower), Math.abs(fLPower));
    // maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
    // maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));
    // maxSpeed = Math.max(maxSpeed, 1.0);

    // turns are determined by the relative speeds of the motor (1.2 --> 1.0)
    // makes sure values being sent to motors are within the range -1 to 1 (inclusive)
    // fLPower /= maxSpeed;
    // fRPower /= maxSpeed;
    // bLPower /= maxSpeed;
    // bRPower /= maxSpeed;

    frontLeftMotor.setPower(fLPower);
    frontRightMotor.setPower(fRPower);
    backLeftMotor.setPower(bLPower);
    backRightMotor.setPower(bRPower);
  }

  public void driveFieldRelative(
      final double forward,
      final double strafe,
      final double rotate,
      final double robotHeadingDeg) {
    final double theta = Math.toRadians(robotHeadingDeg); // Convert to radians

    // Rotate joystick input by negative robot heading to stay field-aligned
    final double tempForward = forward * Math.cos(theta) - strafe * Math.sin(theta);
    final double tempStrafe = forward * Math.sin(theta) + strafe * Math.cos(theta);

    this.drive(tempForward, tempStrafe, rotate);
  }

  public void drive(final double forward, final double right, final double rotate) {
    //        double fLPower = -forward + right - rotate;
    //        double fRPower = -forward + right + rotate; //-
    //        double bLPower = -forward + right + rotate;
    //        double bRPower = forward + right - rotate; //-
    final double fLPower = -forward + right + rotate;
    final double fRPower = -forward - right - rotate; // -
    final double bLPower = forward - -right + -rotate;
    final double bRPower = -forward + right - rotate; // -

    //        telemetry.addData("fLPower: ", fLPower);
    //        telemetry.addData("fRPower: ", fRPower);
    //        telemetry.addData("bLPower: ", bLPower);
    //        telemetry.addData("bRPower: ", bRPower);
    setPowers(fLPower, fRPower, bLPower, bRPower);
  }

  public double[] getMotorPowers() {
    return new double[] {
      frontLeftMotor.getPower(),
      frontRightMotor.getPower(),
      backLeftMotor.getPower(),
      backRightMotor.getPower()
    };
  }

  public void stop() {
    setPowers(0, 0, 0, 0);
  }

  public void setExactMotorPowers(
      final double fLPower, final double fRPower, final double bLPower, final double bRPower) {
    frontLeftMotor.setPower(fLPower);
    frontRightMotor.setPower(fRPower);
    backLeftMotor.setPower(bLPower);
    backRightMotor.setPower(bRPower);
  }
}
