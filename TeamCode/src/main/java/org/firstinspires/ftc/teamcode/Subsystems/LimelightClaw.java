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
    private Claw claw;

    private final boolean IS_RED_ALLIANCE = true; // Change based on alliance

    @Override
    public void runOpMode() {
        // Initialize subsystems
        limelight = new Limelight(hardwareMap, telemetry, IS_RED_ALLIANCE);
        claw = new Claw(hardwareMap, telemetry);

        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            boolean AimActive = gamepad1.a;

            if (AimActive && limelight.hasTarget()) {
                // Get rotation correction from Limelight
                double rotate = limelight.getRotationCorrection();

                // Rotate the claw servo based on Limelight correction
                double currentPosition = claw.getClawPosition();
                double newPosition = currentPosition + rotate * 0.01; // scale rotation
                claw.setPosition(newPosition);
            }

            // Claw controls
            if (gamepad2.a){
                claw.openClaw();
            } else if (gamepad2.x) {
                claw.closeClaw();
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
