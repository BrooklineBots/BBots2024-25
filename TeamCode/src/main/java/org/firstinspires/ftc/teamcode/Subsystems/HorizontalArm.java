package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.Constants.ArmPosition;

public class HorizontalArm {

    private DcMotor horizontalArm;

    private Telemetry telemetry;

    public HorizontalArm(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;
        horizontalArm = hwMap.dcMotor.get(ArmConstants.HORIZONTAL_ARM_ID);

        //horizontalArm.setDirection(DcMotor.Direction.REVERSE); //might change

        horizontalArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunMode(DcMotor.RunMode mode){
        horizontalArm.setMode(mode);
    }

    private void resetEncoders(){
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToPosition(ArmPosition position){
        int targetPosition = position.encoderTicks;

        if(targetPosition < ArmConstants.VERTICAL_MIN_POSITION || targetPosition > ArmConstants.VERTICAL_MAX_POSITION){
            stop();
        }

        horizontalArm.setTargetPosition(targetPosition);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalArm.setPower(ArmConstants.VERTICAL_MOVE_POWER);
    }

    public void moveOut(double power){
        horizontalArm.setPower(-power);
    }

    public void stop(){
        horizontalArm.setPower(0);
    }

    public boolean isBusy(){
        return horizontalArm.isBusy();
    }

    public void update(){
        int middlePos = horizontalArm.getCurrentPosition();

        if(middlePos < ArmConstants.VERTICAL_MIN_POSITION || middlePos > ArmConstants.VERTICAL_MAX_POSITION){
            stop();
        }
    }

}
