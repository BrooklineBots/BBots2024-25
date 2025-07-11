package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

@TeleOp(name = "LimeLightClaw TeleOp", group = "Subsystems")
public class LimelightClaw extends LinearOpMode {

    private Limelight limelight;
    private Servo clawRotate;
    private Servo claw;

    private final boolean IS_RED_ALLIANCE = true; // Change based on alliance

    @Override
    public void runOpMode() {
        // Initialize subsystems
        limelight = new Limelight(hardwareMap, telemetry, IS_RED_ALLIANCE);
        clawRotate = hardwareMap.get(Servo.class, "clawRotateServo");
        clawRotate = hardwareMap.get(Servo.class, "clawServo");

        double tv = limelight.limelightTable.getEntry("tv").getDouble(0);

        clawRotate.setPosition(0);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            boolean AimActive = gamepad1.a;

            telemetry.addData(limelight.tv);
            if (AimActive && limelight.hasTarget()) {
                // Get rotation correction from Limelight
                double rotate = limelight.getRotationCorrection();

                // Rotate the claw servo based on Limelight correction
                double currentPosition = clawRotate.getPosition();
                double newPosition = currentPosition + rotate * 0.01; // scale rotation
                clawRotate.setPosition(newPosition);
            }

            // Claw controls
            if (gamepad2.a){
                claw.setPosition(0);
            } else if (gamepad2.x) {
                claw.setPosition(0);
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
