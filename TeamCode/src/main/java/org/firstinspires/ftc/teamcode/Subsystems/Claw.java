package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw {

    private Servo claw;

    public Claw(HardwareMap hwMap, Telemetry telemetry){
        claw = hwMap.get(Servo.class, "servo1");
        claw.setDirection(Servo.Direction.REVERSE);
    }

    public void openClaw(){
        claw.setPosition(0.0);
    }

    public void closeClaw(){
        claw.setPosition(0.27);
    }

    public void setPosition(double position){
        //TODO *add later*
    }

    public double getPosition(){
        return claw.getPosition();
    }


}
