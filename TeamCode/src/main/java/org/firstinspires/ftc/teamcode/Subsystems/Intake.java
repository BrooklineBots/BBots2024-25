package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake {
    private Servo leftWheel;
    private Servo rightWheel;

    private Telemetry telemetry;

    public Intake(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        leftWheel = hwMap.get(Servo.class, Constants.IntakeConstants.LEFT_WHEEL_ID);
        rightWheel = hwMap.get(Servo.class, Constants.IntakeConstants.RIGHT_WHEEL_ID);

        leftWheel.setDirection(Servo.Direction.FORWARD);
        rightWheel.setDirection(Servo.Direction.REVERSE);
    }

    public void collect(){
        setPos(Constants.IntakeConstants.SERVO_POWER);
    }

    public void release(){
        setPos(-Constants.IntakeConstants.SERVO_POWER);
    }

    public void setPos(double position){
        leftWheel.setPosition(position);
        rightWheel.setPosition(position);
    }

    public boolean voltageSpike(){
        return false;
    }

    public void stop(){
        leftWheel.setPosition(0);
        rightWheel.setPosition(0);
    }


}
