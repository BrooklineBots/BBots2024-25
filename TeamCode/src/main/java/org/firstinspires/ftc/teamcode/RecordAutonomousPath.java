package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "RecordAutonomousPath")
public class RecordAutonomousPath extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor tallLinearActuator;
    Servo servo1;

    ElapsedTime timer = new ElapsedTime();
    private List<MovementData> recordedMovements = new ArrayList<>();
    private boolean recording = false;

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
    }

    @Override
    public void loop() {
        if (gamepad1.a && !recording) {
            recording = true;
            timer.reset();
            telemetry.addData("Recording", "Started");
        }

        double movement = 0;
        double strafe = 0;
        double rotation = 0;

        // Handle forward/backward movement
        if (gamepad1.left_stick_y != 0) {
            movement = gamepad1.left_stick_y;
        }

        // Handle left/right strafing
        if (gamepad1.left_stick_x != 0) {
            strafe = gamepad1.left_stick_x;
        }

        // Handle rotation
        if (gamepad1.right_stick_x != 0) {
            rotation = gamepad1.right_stick_x > 0 ? 0.5 : -0.5;
        }

        // Apply movement
        if (movement != 0 || strafe != 0 || rotation != 0) {
            drive(movement, strafe, rotation);
            if (recording) {
                recordedMovements.add(new MovementData(movement, strafe, rotation, timer.seconds()));
            }
        } else {
            stopMotors();
            if (recording) {
                recordedMovements.add(new MovementData(0, 0, 0, timer.seconds()));
            }
        }

        // Handle linear actuator
        if (gamepad1.right_bumper) {
            moveActuator(0.5);
        } else if (gamepad1.left_bumper) {
            moveActuator(0);
        } else if (gamepad1.y) {
            moveActuator(-0.5);
        }

        if (recording && timer.seconds() > 45) {
            recording = false;
            saveRecordedMovements();
            telemetry.addData("Recording", "Completed and saved");
        }

        telemetry.addData("Recording Status", recording ? "Recording" : "Not Recording");
        telemetry.addData("Time", "%.1f seconds", timer.seconds());
        telemetry.update();
    }

    private void drive(double movement, double strafe, double rotation) {
        // For strafing, we use the original values from the 'left' method
        double strafeValue = strafe != 0 ? 0.3 : strafe;

        // Calculate power for each motor
        double frontLeftPower = movement + (strafe != 0 ? strafeValue : -strafe) + rotation;
        double backLeftPower = movement + (strafe != 0 ? -strafeValue : strafe) + rotation;
        double frontRightPower = movement + (strafe != 0 ? strafeValue : -strafe) - rotation;
        double backRightPower = movement + (strafe != 0 ? -strafeValue : strafe) - rotation;

        // Apply power to motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    private void moveActuator(double power) {
        tallLinearActuator.setPower(power);
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void saveRecordedMovements() {
        try (FileWriter writer = new FileWriter("/sdcard/FIRST/recordedPath.csv")) {
            for (MovementData data : recordedMovements) {
                writer.write(data.toString() + "\n");
            }
            telemetry.addData("File", "Path saved to /sdcard/FIRST/recordedPath.csv");
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to save path: " + e.getMessage());
        }
    }

    private static class MovementData {
        double forward;
        double right;
        double rotate;
        double timestamp;

        MovementData(double forward, double right, double rotate, double timestamp) {
            this.forward = forward;
            this.right = right;
            this.rotate = rotate;
            this.timestamp = timestamp;
        }

        @Override
        public String toString() {
            return forward + "," + right + "," + rotate + "," + timestamp;
        }
    }
}