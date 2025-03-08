package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Claw {

    private final double MAX_POSITION = 1;
    private final double MIN_POSITION = 0;
    private Servo claw;

    private Telemetry telemetry;

    public Claw(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        claw = hwMap.get(Servo.class, Constants.ClawConstants.CLAW_SERVO_ID);
        claw.setDirection(Servo.Direction.REVERSE);
        telemetry.addData("Position:", claw.getPosition());
    }

    public void openClaw(){
        setPosition(Constants.ClawConstants.OPEN_POSITION);
    }

    public void closeClaw(){
        setPosition(Constants.ClawConstants.CLOSE_POSITION);
    }

    public void setPosition(double position){
        if(position >= MIN_POSITION && position <= MAX_POSITION){
            claw.setPosition(position);
        }
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public double getPosition(){
        return claw.getPosition();
    }




}
