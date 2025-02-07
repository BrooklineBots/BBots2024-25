package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DriveOpMode6;

@Autonomous(name = "Drive Right", group = "Autonomous")
public class DriveRightAuto extends LinearOpMode {
    private DriveOpMode6 driver;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() {
        initMotors(hardwareMap);
        waitForStart();
        right(-1);
        sleep(5000);
        right(0);

    }

    public void right(double power) {
        frontLeftMotor.setPower(-power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(-power);
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


}
