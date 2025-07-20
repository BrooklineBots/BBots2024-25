package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.ArmPosition;
import org.firstinspires.ftc.teamcode.Constants.OuttakeArmPosition;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalArm;

@Autonomous(name = "Smart Auto", group = "Competition")
public class SmartAuto extends LinearOpMode {
  // Subsystems
  private MecanumDrive drive;
  private VerticalArm verticalArm;
  private Outtake outtake;
  private OuttakeArm outtakeArm;
  private final ElapsedTime timer = new ElapsedTime();

  @Override
  public void runOpMode() {
    // Initialize subsystems
    drive = new MecanumDrive(hardwareMap, telemetry);
    verticalArm = new VerticalArm(hardwareMap, telemetry);
    outtake = new Outtake(hardwareMap, telemetry);
    outtakeArm = new OuttakeArm(hardwareMap, telemetry);

    drive.resetYaw();

    waitForStart();

    // TODO: Adjust distances and angles based on field setup, these are all guesses
    drive.driveDistance(0.5); // Drive forward 0.5 meters

    drive.rotate(90); // Rotate 90 degrees left (counter-clockwise)

    drive.driveDistance(2.0); // Drive forward 2 meters

    drive.rotate(-90); // Rotate 90 degrees right (clockwise)

    // Raise arm to max height
    verticalArm.goToPosition(ArmPosition.SCORE_HIGH_BAR);
    timer.reset();
    while (timer.milliseconds() < Constants.AutoConstants.ARM_DELAY_MS && opModeIsActive()) {
      telemetry.addData("Raising Arm", verticalArm.getCurrentPosition()[0]);
    }

    // Drive forward 1 meter slowly
    drive.driveDistance(1.0, Constants.AutoConstants.SLOW_DRIVE_SPEED);

    // Score piece
    outtakeArm.goToPosition(OuttakeArmPosition.SCORE_HIGH_BAR_POSITION);
    sleep(500); // Allow arm movement
    outtake.openClaw();
    sleep(Constants.AutoConstants.SCORE_DELAY_MS);

    // Raise arm slightly higher
    verticalArm.setArmPowers(0.2); // Small adjustment
    sleep(500);
    verticalArm.stop();

    // Drive backward 1 meter slowly
    drive.driveDistance(-1.0, Constants.AutoConstants.SLOW_DRIVE_SPEED);

    // Return arm to stowed position
    verticalArm.goToPosition(ArmPosition.STOWED);
    timer.reset();
    while (timer.milliseconds() < Constants.AutoConstants.ARM_DELAY_MS && opModeIsActive()) {
      // Wait for arm to stow
      telemetry.addData("Stowing Arm", verticalArm.getCurrentPosition()[0]);
    }

    // Return to start
    drive.rotate(90); // Face original starting orientation
    drive.driveDistance(-2.0);
    drive.rotate(-90);
    drive.driveDistance(-0.5);

    // Final stop
    drive.stop();
  }
}
