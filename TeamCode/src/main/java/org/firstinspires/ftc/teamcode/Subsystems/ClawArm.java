package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.*;

public class ClawArm {

    private final double MAX_POSITION = 1;
    private final double MIN_POSITION = 0;
    private ClawArmPosition goalPosition;
    private final Servo clawArm;

    private final Telemetry telemetry;

    public ClawArm(final HardwareMap hwMap, final Telemetry telemetry) {
        this.telemetry = telemetry;
        clawArm = hwMap.get(Servo.class, ClawArmConstants.CLAW_ARM_SERVO_ID);
        //clawArm.setDirection(Servo.Direction.REVERSE);
        goalPosition = ClawArmPosition.TRANSFER_POSITION;
        telemetry.addData("Position:", clawArm.getPosition());
    }

    public void moveToTransfer(){
        setPosition(ClawArmPosition.TRANSFER_POSITION.position);
        goalPosition = ClawArmPosition.TRANSFER_POSITION;
    }

    public void scoreHighBar(){
        setPosition(ClawArmPosition.SCORE_HIGH_BAR_POSITION.position);
        goalPosition = ClawArmPosition.SCORE_HIGH_BAR_POSITION;
    }
    public void scoreHighBucket(){
        setPosition(ClawArmPosition.SCORE_HIGH_BUCKET_POSITION.position);
        goalPosition = ClawArmPosition.SCORE_HIGH_BUCKET_POSITION;
    }
    public void scoreLowBucket() {
        setPosition(ClawArmPosition.SCORE_LOW_BUCKET_POSITION.position);
        goalPosition = ClawArmPosition.SCORE_LOW_BUCKET_POSITION;
    }

    public void moveToPickup(){
        setPosition(ClawArmPosition.PICKUP_POSITION.position);
        goalPosition = ClawArmPosition.PICKUP_POSITION;
    }


    public ClawArmPosition getGoalPosition() {
        return goalPosition;
    }

    public void setPosition(final double position) {
        if (position >= MIN_POSITION && position <= MAX_POSITION) {
            clawArm.setPosition(position);
        }
    }

    public double getClawArmPosition() {
        return clawArm.getPosition();
    }
}
