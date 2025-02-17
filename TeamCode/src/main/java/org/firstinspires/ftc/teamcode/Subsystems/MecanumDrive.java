package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import java.util.Random;


public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private Telemetry telemetry;
    private IMU imu;
    public MecanumDrive(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
        frontRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
        backLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
        backRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);
        this.telemetry = telemetry;

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));

        // Generate a random number between 10 and 100 (inclusive)


        telemetry.addData("IMU Angle:", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        this.telemetry = telemetry;
    }

    private void setPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        //finds the highest speed b/w 1 and fL then that and fR and so on
        double maxSpeed = Math.max(Math.abs(fRPower), Math.abs(fLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(bRPower));

        //turns are determined by the relative speeds of the motor (1.2 --> 1.0)
        //makes sure values being sent to motors are within the range -1 to 1 (inclusive)
        fLPower /= maxSpeed;
        fRPower /= maxSpeed;
        bLPower /= maxSpeed;
        bRPower /= maxSpeed;

        frontLeftMotor.setPower(fLPower);
        frontRightMotor.setPower(fRPower);
        backLeftMotor.setPower(-bLPower);
        backRightMotor.setPower(bRPower);
    }

    public void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) / 180;
        telemetry.addData("IMU Angle:", robotAngle);
        telemetry.update();

        // Convert to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // Convert back to Cartesian coordinates
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        this.drive(newForward, -newRight, -rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double fLPower = forward + right + rotate;
        double fRPower = forward - right - rotate; //-
        double bLPower = forward - right + rotate;
        double bRPower = forward + right - rotate; //-

        setPowers(fLPower, fRPower, bLPower, bRPower);
    }

    public void stopMotors() {
        setPowers(0, 0, 0, 0);
    }
}
