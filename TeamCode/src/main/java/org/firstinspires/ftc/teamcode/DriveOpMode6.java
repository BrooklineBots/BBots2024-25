package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testDrive2")
public class DriveOpMode6 extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor tallLinearActuator;

    Servo servo1;


    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        tallLinearActuator = hardwareMap.dcMotor.get("tall_linear_actuator");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        tallLinearActuator.setDirection(DcMotor.Direction.REVERSE);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo1.setDirection(Servo.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0) {
            forward(gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_y < 0) {
            forward(gamepad1.left_stick_y);
        } else if (gamepad1.left_stick_x > 0) {
            right(gamepad1.left_stick_x);
        } else if (gamepad1.left_stick_x < 0) {
            right(gamepad1.left_stick_x);
        } else if (gamepad1.right_stick_x < 0) {
            rotateRight(gamepad1.right_stick_x);
        } else if (gamepad1.right_stick_x > 0) {
            rotateRight(gamepad1.right_stick_x);
        } else if (gamepad2.left_stick_y > 0) {
            moveUp(gamepad2.left_stick_y);
        } else if (gamepad2.left_stick_button) {
            moveUp(0);
        } else if (gamepad2.left_stick_y < 0) {
            moveUp(gamepad2.left_stick_y);
        } else if (gamepad1.a) {
            servo1.setPosition(0.27);
        } else if (gamepad1.b) {
            servo1.setPosition(0.0);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        telemetry.addData("Position:", servo1.getPosition());
    }

    public void forward(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);  //1
        backRight.setPower(power); //1
    }

    public void right(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(-power);
    }

    public void rotateRight(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void moveUp(double power) {
        tallLinearActuator.setPower(-power);

    }


}
