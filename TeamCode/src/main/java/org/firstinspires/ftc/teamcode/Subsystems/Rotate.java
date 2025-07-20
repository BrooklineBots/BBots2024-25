package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.Constants.IntakePosition;

public class Rotate {

    private Servo clawServo;
    private Servo flipServo;
    private CRServo rotateServo;
    private Telemetry telemetry;
    private Limelight limelight;

    private static final double ALIGNMENT_TOLERANCE = 1.0; // degrees

    public Rotate(HardwareMap hwMap, Telemetry telemetry, Limelight limelight) {
        this.telemetry = telemetry;
        this.limelight = limelight;

        clawServo = hwMap.get(Servo.class, IntakeConstants.CLAW_INTAKE_SERVO_ID);
        flipServo = hwMap.get(Servo.class, IntakeConstants.INTAKE_FLIP_SERVO_ID);
        rotateServo = hwMap.get(CRServo.class, IntakeConstants.CLAW_ROTATION_SERVO_ID);
        rotateServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void open() {
        clawServo.setPosition(IntakePosition.CLAW_OPEN_POSITION.position);
        telemetry.addLine("Claw: OPEN");
    }

    public void close() {
        clawServo.setPosition(IntakePosition.CLAW_CLOSE_POSITION.position);
        telemetry.addLine("Claw: CLOSED");
    }

    public void flipTo(IntakePosition position) {
        flipServo.setPosition(position.position);
        telemetry.addData("Flip to", position.name());
    }

    public void rotate(double power) {
        rotateServo.setPower(power);
        telemetry.addData("Rotate Power", power);
    }

    public void stopRotate() {
        rotateServo.setPower(0);
        telemetry.addLine("Claw Rotation: STOPPED");
    }

    public void autoAlign() {
        if (!limelight.hasTarget()) {
            telemetry.addLine("Vision: no target");
            stopRotate();
            return;
        }

        double tx = limelight.getTx();
        if (Math.abs(tx) <= ALIGNMENT_TOLERANCE) {
            telemetry.addLine("Claw: Aligned");
            stopRotate();
        } else {
            double power = limelight.getClawAlignmentPower();
            rotate(power);
        }
    }
}
