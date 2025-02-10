package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private IMU imu;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
        frontRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
        backLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
        backRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    private void setPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        //finds the highest speed b/w 1 and fL then that and fR and so on
        double maxSpeed = 1;
        maxSpeed = Math.max(maxSpeed, Math.abs(fLPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(fRPower));
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
        backLeftMotor.setPower(bLPower);
        backRightMotor.setPower(bRPower);
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        //convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        this.drive(newForward, newRight, rotate);

    }

    public void drive(double forward, double right, double rotate) {
        double fLPower = forward + right + rotate;
        double fRPower = forward - right - rotate;
        double bLPower = forward - right + rotate;
        double bRPower = forward + right - rotate;

        setPowers(fLPower, fRPower, bLPower, bRPower);
    }

    public void stopMotors() {
        setPowers(0, 0, 0, 0);
    }
}
