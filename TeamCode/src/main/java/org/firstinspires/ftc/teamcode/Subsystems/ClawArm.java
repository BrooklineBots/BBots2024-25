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

    private long startTimeNs = -1;

    public ClawArm(final HardwareMap hwMap, final Telemetry telemetry) {
        this.telemetry = telemetry;
        clawArm = hwMap.get(Servo.class, ClawArmConstants.CLAW_ARM_SERVO_ID);

        telemetry.addData("Position:", clawArm.getPosition());
    }

    public void goToPosition(ClawArmPosition position){
        setPosition(position.position);
        goalPosition = position;
    }
    public void moveToTransfer(){

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
