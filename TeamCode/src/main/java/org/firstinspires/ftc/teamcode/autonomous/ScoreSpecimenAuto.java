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

@Autonomous(name = "ScoreSpecimen", group = "Autonomous")
public class ScoreSpecimenAuto extends LinearOpMode {
  private MecanumDrive drive;
  private HorizontalExtension horizontalExtension;
  private Intake intake;
  private HomingSensor touchSensor;
  private Outtake outtake;
  private OuttakeArm outtakeArm;
  private VerticalArm arm;

  private final boolean outtakeArmInPosition = false;

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

      outtakeArm.goToPosition(Constants.OuttakeArmPosition.AUTONOMOUS_POSITION);
      intake.goToPositionFlip(Constants.IntakePosition.FLIP_TRANSFER_POSITION);
      if (!touchSensor.isPressed()) {
        horizontalExtension.home();
      } else {
        horizontalExtension.stopServos();
      }

      // score first specimen
      backward(45);
      strafeRight(54);
      rotateRight(5);
      goToHighBar(110); // 90
      driveWait((float) 0.75);
      backward(79); // 75 //67;
      scoreSpecimen(45);
      rotateRight(20);

      // score second specimen
      forward(40);
      rotateLeft(590);
      strafeRight(130); // 90//80 //100 //130 //150
      forward(160); // 255

      strafeRight(20); // 25
      backward(250); // 280 //220
      driveWait(3 / 2);
      forward(150); // 100
      sleep(2000);
      driveWait(1);

      moveToPickup(60);
      rotateRight(25);
      strafeRight(20);
      slowBackward(184);
      sleep(2000);
      sleep(100);
      grabSpecimen(29);
      forward(10);
      strafeRight(20);
      backward(10);
      sleep(100);

      // backward(30);

      forward(20);
      rotateRight(605);
      strafeRight(170);
      rotateLeft(1);
      goToHighBar(61);
      backward(100);
      scoreSpecimen(45);
    }
  }

  private void forward(final long cm) {
    drive.driveRobotCentricFast(0.9, 0, 0);
    final long time = cm * (2000 / 603);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void backward(final long cm) {
    drive.driveRobotCentricFast(-0.9, 0, 0);
    final long time = cm * (2000 / 603);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void slowBackward(final long cm) {
    drive.driveRobotCentricFast(-0.5, 0, 0);
    final long time = cm * (2000 / 603);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void strafeRight(final long cm) {
    drive.driveRobotCentricFast(0, 0.9, 0);
    final long time = cm * (1440 / 137);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void strafeLeft(final long cm) {
    drive.driveRobotCentricFast(0, -0.9, 0);
    final long time = cm * (1440 / 137);
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void rotateRight(final long time) { // In Milliseconds
    drive.driveRobotCentricFast(0, 0, 0.9);
    /*
    90 degrees - 227
    180 degrees -
     */
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void rotateLeft(final long time) {
    drive.driveRobotCentricFast(0, 0, -0.9);
    /*
    90 degrees - 230
    180 degrees - 595
     */
    sleep(time);
    drive.stopMotors();
    sleep(100);
  }

  private void driveWait(final float seconds) {
    drive.stopMotors();
    final float time = seconds * 1000;
    sleep((long) time);
  }

  private void grabSpecimen(final long cm) {
    outtake.closeClaw();
    sleep(1000);
    outtake.closeClaw();
    final long time = cm * (1000 / 121);
    arm.setArmPowers(0.9);
    sleep(time);
    arm.setArmPowers(0);
  }

  private void moveToPickup(final long cm) {
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.PICKUP_POSITION);
    sleep(2000);
    outtake.openClaw();
    final long time = cm * (1000 / 121);
    arm.setArmPowers(-0.9);
    sleep(time);
    arm.setArmPowers(0);
  }

  private void goToHighBar(final long cm) {
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
    sleep(500);
    outtake.closeClaw();
    sleep(100);
    outtake.closeClaw();
    final long time = cm * (1000 / 121);
    arm.setArmPowers(0.9);
    sleep(time);
    arm.setArmPowers(0);
  }

  private void scoreSpecimen(final long cm) {
    outtake.closeClaw();
    final long time = cm * (1000 / 121);
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
    arm.setArmPowers(-0.9);
    sleep(time);
    arm.setArmPowers(0);
    sleep(100);
    outtake.openClaw();
    sleep(500);
  }
}
