package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.Set;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name="Mecanum Drive Autonomous", group="Autonomous")
public class MecanumDriveAutonomous extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        initMotors(hardwareMap);
        waitForStart();


        // Test the wheels
        if(false) {
            testFrontLeft(1.0, 1000);
            testFrontRight(1.0, 1000);
            testBackLeft(1.0, 1000);
            testBackRight(1.0, 1000);
        }

        // Move in square  for 1 second (adjust as needed)
        if(true) {
            driveForward(0.1, 1000); // Power, duration in milliseconds
            driveRight(0.66, 1000); // Power, duration in milliseconds
           // driveBackward(1.0, 1000); // Power, duration in milliseconds
            //driveLeft(0.66, 1000); // Power, duration in milliseconds
            // do drive left if starting on left, drive right if starting on right side
        }
    }

    public void initMotors(HardwareMap hwMap) {
        frontLeftMotor = hwMap.dcMotor.get("front_left_motor");
        frontRightMotor = hwMap.dcMotor.get("front_right_motor");
        backLeftMotor = hwMap.dcMotor.get("back_left_motor");
        backRightMotor = hwMap.dcMotor.get("back_right_motor");

        //frontLeftMotor.setDirection(DcMotor.Direction.REVERSE); bc not work
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void testFrontLeft(double power, long duration) {
        setPowers(power, 0,0,0);
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
        sleep(duration);
    }

    public void testFrontRight(double power, long duration) {
        setPowers(0,power,0,0);
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
        sleep(duration);
    }

    public void testBackLeft(double power, long duration) {
        setPowers(0, 0,power,0);
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
        sleep(duration);
    }

    public void testBackRight(double power, long duration) {
        setPowers(0,0,0,power);
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
        sleep(duration);
    }

    public void driveForward(double power, long duration) {
        setPowers(power, power, power, power);
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
    }

    public void driveRight(double power, long duration) {
         //Set motor powers for left strafe
        setPowers(power, power, -power, -power); // Adjust motor powers for left movement
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
    }

    public void driveLeft(double power, long duration) {
        // Set motor powers for right strafe
        setPowers(-power, -power, power, power); // Adjust motor powers for right movement
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
    }

    public void driveBackward(double power, long duration) {
        // Set motor powers for backward movement
        setPowers(-power, -power, -power, -power); // Set all motors to negative power
        sleep(duration);
        setPowers(0, 0, 0, 0); // Stop the motors
    }
    private void setPowers(double fLPower, double fRPower, double bLPower, double bRPower) {
        frontLeftMotor.setPower(fLPower);
        frontRightMotor.setPower(fRPower);
        backLeftMotor.setPower(bLPower);
        backRightMotor.setPower(bRPower);
    }
}
