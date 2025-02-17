package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Arm {

    private DcMotor arm;

    private Telemetry telemetry;

    public Arm(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        arm = hwMap.dcMotor.get(Constants.ArmConstants.ARM_MOTOR_ID);
        arm.setDirection(DcMotor.Direction.REVERSE);
    }

    public void moveUp(double power){
        arm.setPower(-power);
    }

    public void stop(){
        arm.setPower(0);
    }


}
