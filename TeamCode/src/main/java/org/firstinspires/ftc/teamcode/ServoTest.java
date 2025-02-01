package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class ServoTest extends OpMode{
    Servo servo1;
    Servo servo2;
    @Override
    public void init(){
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        
    }
    
    @Override
    public void loop(){
        if(gamepad1.a){
            servo1.setPosition(0.8);
            //servo2.setPosition(0.3);
        }
        // else if(gamepad1.b){
        //     servo1.setPosition(servo1.getPosition() - 1);
        //     servo2.setPosition(servo2.getPosition() + 1);
        // }
        else if(gamepad1.b){
            servo1.setPosition(0);
            servo2.setPosition(0);
        }
        
        telemetry.addData("Position:", servo1.getPosition());
    }
}
