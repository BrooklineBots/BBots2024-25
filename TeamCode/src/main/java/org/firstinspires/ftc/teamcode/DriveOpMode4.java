/*
 * teamcode - DriveOpMode4
 *
 * February 2025
 *
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testDrive2")
public class DriveOpMode4 extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor tallLinearActuator;

    public static boolean isWithinTolerance(
            double currentValue, double targetValue, double tolerance) {
        return Math.abs(currentValue - targetValue) <= tolerance;
    }

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        tallLinearActuator = hardwareMap.dcMotor.get("tall_linear_actuator");
        tallLinearActuator.setDirection(DcMotor.Direction.REVERSE);
    }

    private double getPowerValue(double yValue, double xValue, double tolerance, boolean negate) {
        double power;
        boolean yTolerance = isWithinTolerance(yValue, 0, tolerance);
        boolean xTolerance = isWithinTolerance(xValue, 0, tolerance);
        if (yTolerance && xTolerance) {
            power = Math.hypot(yValue, xValue);
        } else if (yTolerance) {
            power = yValue;
        } else if (xTolerance) {
            power = xValue;
        } else {
            power = 0;
        }
        return negate ? -power : power;
    }

    @Override
    public void loop() {
        double frontLeftPower = getPowerValue(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.1, true);
        double frontRightPower = getPowerValue(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.1, false);
        double backLeftPower = getPowerValue(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.1, false);
        double backRightPower = getPowerValue(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.1, false);


//        if (isWithinTolerance(gamepad1.left_stick_x, 0.5, 0.25)) {
//            frontRightPower = 0;
//            backLeftPower = 0;
//        } else if (isWithinTolerance(gamepad1.left_stick_x, -0.5, 0.25)) {
//            frontLeftPower = 0;
//            backRightPower = 0;
//        }

//        AutonomousRecorder recorder = new AutonomousRecorder();
//        recorder.setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        if (isWithinTolerance(gamepad2.left_stick_y, 0, 0.1)) {
            tallLinearActuator.setPower(gamepad2.left_stick_y);
        }


    }


    public void setMotors(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
//
//
//    public void stop() {
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//    }
//
//    public void forward(double power) {
//        frontLeft.setPower(power);
//        backLeft.setPower(power);
//        frontRight.setPower(power);
//        backRight.setPower(power);
//    }
//
//    public void right(double power) {
//        frontLeft.setPower(-power);
//        backLeft.setPower(power);
//        frontRight.setPower(-power);
//        backRight.setPower(power);
//    }
//
//    public void rotateRight(double power) {
//        frontLeft.setPower(-power);
//        backLeft.setPower(-power);
//        frontRight.setPower(power);
//        backRight.setPower(power);
//    }
//
//    public void moveArm(double power) {
//        tallLinearActuator.setPower(power);
//    }
//
//    public void stopArm(double power) {
//        tallLinearActuator.setPower(power);
//    }


}
