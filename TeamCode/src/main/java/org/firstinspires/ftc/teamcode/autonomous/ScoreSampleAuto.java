package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.HomingSensor;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalArm;

@Autonomous(name = "ScoreSample", group = "Autonomous")
public class ScoreSampleAuto extends LinearOpMode {
  private MecanumDrive drive;
  private HorizontalExtension horizontalExtension;
  private Intake intake;
  private HomingSensor touchSensor;
  private Outtake outtake;
  private OuttakeArm outtakeArm;
  private VerticalArm arm;

  private boolean isDriving = false;
  private boolean hasDefaulted = false;

  @Override
  public void runOpMode() throws InterruptedException {
    drive = new MecanumDrive(hardwareMap, telemetry);
    horizontalExtension = new HorizontalExtension(hardwareMap, telemetry);
    intake = new Intake(hardwareMap, telemetry);
    touchSensor = new HomingSensor(hardwareMap, telemetry);
    outtake = new Outtake(hardwareMap, telemetry);
    outtakeArm = new OuttakeArm(hardwareMap, telemetry);
    arm = new VerticalArm(hardwareMap, telemetry);

    arm.setModeRunWithoutEncoder();

    waitForStart();

    if (opModeIsActive()) {
      if (!hasDefaulted) {
        intake.goToPositionFlip(Constants.IntakePosition.FLIP_TRANSFER_POSITION);
        outtakeArm.goToPosition(Constants.OuttakeArmPosition.AUTONOMOUS_POSITION);
        hasDefaulted = true;
      }
      if (isDriving) {
        if (!touchSensor.isPressed()) {
          horizontalExtension.home();
        } else {
          horizontalExtension.stopServos();
        }
      }
    }
  }

  private void forward(long cm) {
    isDriving = true;
    drive.driveRobotCentricFast(0.9, 0, 0);
    long time = cm * (2000 / 603);
    sleep(time);
    drive.stopMotors();
    sleep(100);
    isDriving = false;
  }

  private void backward(long cm) {
    drive.driveRobotCentricFast(-0.9, 0, 0);
    long time = cm * (2000 / 603);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void strafeRight(long cm) {
    drive.driveRobotCentricFast(0, 0.9, 0);
    long time = cm * (1440 / 137);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void strafeLeft(long cm) {
    drive.driveRobotCentricFast(0, -0.9, 0);
    long time = cm * (1440 / 137);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void rotateRight(long time) {
    drive.driveRobotCentricFast(0, 0, 0.9);
    /*
    90 degrees - 227
    180 degrees -
     */
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void rotateLeft(long time) {
    drive.driveRobotCentricFast(0, 0, -0.9);
    /*
    90 degrees - 230
    180 degrees - 595
     */
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void driveWait(long seconds) {
    drive.stopMotors();
    long time = seconds * 1000;
    sleep(time);
  }

  private void grabSpecimen(long cm) {
    outtake.closeClaw();
    sleep(1000);
    outtake.closeClaw();
    long time = cm * (1000 / 121);
    arm.setArmPowers(0.9);
    sleep(time);
    arm.setArmPowers(0);
  }

  private void moveToPickup(long cm) {
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.PICKUP_POSITION);
    sleep(2000);
    outtake.openClaw();
    long time = cm * (1000 / 121);
    arm.setArmPowers(-0.9);
    sleep(time);
    arm.setArmPowers(0);
  }

  private void goToHighBar(long cm) {
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
    sleep(500);
    outtake.closeClaw();
    sleep(100);
    outtake.closeClaw();
    long time = cm * (1000 / 121);
    arm.setArmPowers(0.9);
    sleep(time);
    arm.setArmPowers(0);
  }

  private void scoreSpecimen(long cm) {
    outtake.closeClaw();
    long time = cm * (1000 / 121);
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
    arm.setArmPowers(-0.9);
    sleep(time);
    arm.setArmPowers(0);
    sleep(100);
    outtake.openClaw();
    sleep(500);
  }
}
