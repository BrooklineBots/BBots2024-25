package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ArmPosition;
import org.firstinspires.ftc.teamcode.Constants.ArmConstants;

public class Arm {

    private DcMotor leftArmMotor;
    private DcMotor rightArmMotor;
    private DcMotor middleArmMotor;
    private Telemetry telemetry;

    public Arm(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftArmMotor = hwMap.dcMotor.get(ArmConstants.LEFT_ARM_MOTOR_ID);
        rightArmMotor = hwMap.dcMotor.get(ArmConstants.RIGHT_ARM_MOTOR_ID);
        middleArmMotor = hwMap.dcMotor.get(ArmConstants.MIDDLE_ARM_MOTOR_ID);

        rightArmMotor.setDirection(DcMotor.Direction.REVERSE);

        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middleArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        leftArmMotor.setMode(mode);
        rightArmMotor.setMode(mode);
        middleArmMotor.setMode(mode);
    }

    private void resetEncoders() {
        setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void goToPosition(ArmPosition position) {
        int target = position.encoderTicks;

        if (target < ArmConstants.MIN_POSITION || target > ArmConstants.MAX_POSITION) {
            telemetry.addData("Arm Error", "Target position %s out of bounds", position.name());
            telemetry.update();
            return;
        }

        leftArmMotor.setTargetPosition(target);
        rightArmMotor.setTargetPosition(target);


        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftArmMotor.setPower(ArmConstants.MOVE_POWER);
        rightArmMotor.setPower(ArmConstants.MOVE_POWER);

        telemetry.addData("Arm", "Moving to %s at %d ticks", position.name(), target);
        telemetry.update();
    }

    public void moveUp(double power) {
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmMotor.setPower(-power);
        rightArmMotor.setPower(-power);
        telemetry.addData("Left Arm tick:", leftArmMotor.getCurrentPosition());
        telemetry.addData("Right Arm tick:", rightArmMotor.getCurrentPosition());
        telemetry.update();
    }

    public void moveOut(double power){
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleArmMotor.setPower(-power);
        telemetry.addData("Middle Arm tick:", middleArmMotor.getCurrentPosition());
        telemetry.update();
    }

    public void stop() {
        leftArmMotor.setPower(0);
        rightArmMotor.setPower(0);
        middleArmMotor.setPower(0);
    }

    public boolean isBusy() {
        return leftArmMotor.isBusy() || rightArmMotor.isBusy();
    }

    public void update() {
        int leftPos = leftArmMotor.getCurrentPosition();
        int rightPos = rightArmMotor.getCurrentPosition();

        if (Math.abs(leftPos - rightPos) > ArmConstants.MAX_ALLOWED_DIFFERENCE) {
            stop();
            telemetry.addData("Arm Error", "Motors out of sync by %d ticks", Math.abs(leftPos - rightPos));
            telemetry.update();
        }

        if (leftPos < ArmConstants.MIN_POSITION || leftPos > ArmConstants.MAX_POSITION ||
                rightPos < ArmConstants.MIN_POSITION || rightPos > ArmConstants.MAX_POSITION) {
            stop();
            telemetry.addData("Arm Error", "Position out of bounds: Left %d, Right %d", leftPos, rightPos);
            telemetry.update();
        }
    }
}