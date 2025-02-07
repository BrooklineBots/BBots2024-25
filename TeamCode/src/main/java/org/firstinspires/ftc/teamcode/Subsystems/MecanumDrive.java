package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrive {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    public void init(HardwareMap hardwareMap){
        frontLeftMotor = hardwareMap.dcMotor.get("front_left_motor");
        frontRightMotor = hardwareMap.dcMotor.get("front_right_motor");
        backLeftMotor = hardwareMap.dcMotor.get("back_left_motor");
        backRightMotor = hardwareMap.dcMotor.get("back_right_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPowers(double fLPower, double fRPower, double bLPower, double bRPower){
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

    public void drive(double forward, double right, double rotate){
        double fLPower = forward + right + rotate;
        double fRPower = forward - right - rotate;
        double bLPower = forward - right + rotate;
        double bRPower = forward + right - rotate;

        setPowers(fLPower,fRPower, bLPower, bRPower);
    }
}
