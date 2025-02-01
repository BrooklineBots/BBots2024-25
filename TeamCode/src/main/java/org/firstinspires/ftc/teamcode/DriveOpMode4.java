package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testDrive2")
public class DriveOpMode4 extends OpMode{

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor tallLinearActuator;
    
    @Override
    public void init(){
        frontLeft = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backLeft = hardwareMap.dcMotor.get("back_left_motor");
        backRight = hardwareMap.dcMotor.get("back_right_motor");
        tallLinearActuator = hardwareMap.dcMotor.get("tall_linear_actuator");
        
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        tallLinearActuator.setDirection(DcMotor.Direction.REVERSE);
        //backLeft.setDirection(DcMotor.Direction.REVERSE);
    }
    
    @Override
    public void loop(){
         if(gamepad1.left_stick_y > 0){
            forward(0.8);
        }
        else if(gamepad1.left_stick_y < 0){
            backward(-0.5);
        }
        else if(gamepad1.left_stick_x > 0){
            right(0.5);
        }
        else if(gamepad1.left_stick_x < 0){
            left(-0.5);
        }
        else if(gamepad1.right_stick_x <0){
            rotateLeft(0.5);
        }
        else if(gamepad1.right_stick_x>0){
            rotateRight(0.5);
        }
        else if(gamepad1.right_bumper){
            moveUp(0.5);
        }
        else if(gamepad1.left_bumper){
            stopUp(0);
        }
        else if(gamepad1.y){
            moveDown(0.5);
        }
        else{
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
        
    }
    
    public void forward(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power+1);
        backRight.setPower(power+1);
    }
    
    public void backward(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    
    public void right(double power){
        frontLeft.setPower(-power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }
    
    public void left(double power){
        frontLeft.setPower(0.3);
        backLeft.setPower(-0.3);
        frontRight.setPower(0.3);
        backRight.setPower(-0.3);
    }
    
    public void rotateLeft(double power){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);
    }
    public void rotateRight(double power){
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }
    public void moveUp(double power){
        tallLinearActuator.setPower(power);

    }
    public void stopUp(double power){
        tallLinearActuator.setPower(power);
    }
    public void moveDown(double power){
        tallLinearActuator.setPower(-power);
    }
    

}
