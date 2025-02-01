package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "realTestDrive")
public class DriveOpMode4 extends OpMode {

    final double TOLERANCE = 0.2; // Tolerance threshold for enabling strafing
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor tallLinearActuator;
    boolean strafeMode = false;

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

        // Reverse necessary motors for proper movement
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Forward/Backward
        double x = gamepad1.left_stick_x;  // Left/Right Strafing
        double rotation = gamepad1.right_stick_x; // Rotation

        // Enable strafe mode if lateral movement is significantly greater than forward/backward movement
        strafeMode = Math.abs(x) > Math.abs(y) + TOLERANCE;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        if (strafeMode) {
            // Mecanum Strafe Mode
            frontLeftPower = x;
            frontRightPower = -x;
            backLeftPower = -x;
            backRightPower = x;
        } else {
            // Normal Drive Mode
            frontLeftPower = y + rotation;
            frontRightPower = -(y - rotation);
            backLeftPower = -(y + rotation);
            backRightPower = -(y - rotation);
        }

        // Ensure values remain within the range -1 to 1
        frontLeftPower = Math.max(-1, Math.min(1, frontLeftPower));
        frontRightPower = Math.max(-1, Math.min(1, frontRightPower));
        backLeftPower = Math.max(-1, Math.min(1, backLeftPower));
        backRightPower = Math.max(-1, Math.min(1, backRightPower));

        setMotors(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        // Control the linear actuator
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            tallLinearActuator.setPower(gamepad2.left_stick_y);
        } else {
            tallLinearActuator.setPower(0);
        }
    }

    public void setMotors(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}
