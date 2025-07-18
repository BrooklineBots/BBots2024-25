package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants.IntakePosition;
import org.firstinspires.ftc.teamcode.autonomous.Limelight;
import org.firstinspires.ftc.teamcode.autonomous.LimelightClaw;

/**
 * Autonomous version of LimelightClaw that auto-aligns to a target and grasps it.
 */
@Autonomous(name = "LimelightClawAuto", group = "Subsystems")
public class LimelightClawTeleOp extends LinearOpMode {

    private Limelight limelight;
    private LimelightClaw claw;
    private boolean isRedAlliance = true;

    @Override
    public void runOpMode() {
        // Initialize subsystems
        limelight = new Limelight(hardwareMap, telemetry, isRedAlliance);
        claw = new LimelightClaw(hardwareMap, telemetry, limelight);

        // Start vision
        limelight.start();

        telemetry.addLine("LimelightClawAuto initialized.");
        telemetry.update();

        waitForStart();

        // Flip claw to pickup position
        claw.flipTo(IntakePosition.FLIP_PICKUP_POSITION);
        sleep(500);

        // Try aligning to the target for a fixed amount of time
        long startTime = System.currentTimeMillis();
        long alignDuration = 2000; // ms

        while (opModeIsActive() && System.currentTimeMillis() - startTime < alignDuration) {
            claw.autoAlign();
            telemetry.update();
        }

        // Stop rotation
        claw.stopRotate();

        // Close claw to grab object
        claw.close();
        sleep(500);

        // Flip claw to transfer position
        claw.flipTo(IntakePosition.FLIP_TRANSFER_POSITION);
        sleep(500);

        // Stop vision
        limelight.stop();
    }
}
