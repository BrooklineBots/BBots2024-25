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
    }

    public boolean collect(){
        
    }

}
