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

@Autonomous(name = "ScoreSample_16093", group = "Autonomous")
public class ScoreSample_16093 extends LinearOpMode {
  private MecanumDrive drive;
  private HorizontalExtension horizontalExtension;
  private Intake intake;
  private HomingSensor touchSensor;
  private Outtake outtake;
  private OuttakeArm outtakeArm;
  private VerticalArm arm;

  private boolean outtakeArmInPosition = false;

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

      // score first sample
      //            backward(45);
      //            strafeRight(54);
      //            rotateRight(1);
      scorePreloadSample();
      //            driveWait(2);
      //            preSamplePark();
      //            driveWait(1);
      //            samplePark();
      //            scoreSpecimen(45);

      //            //score second specimen
      //            forward(35);
      //            rotateLeft(630);
      //            strafeRight(150);
      //            forward(255);
      //            strafeRight(25);
      //            backward(220);
      //            driveWait(3/2);
      //            forward(100);
      //            sleep(2000);
      //            driveWait(1);
      //            moveToPickup(60);
      //            rotateRight(25);
      //            backward(90);
      //            sleep(2000);
      //            sleep(100);
      //            grabSpecimen(29);
      //            sleep(100);

      // backward(30);

    }
  }

  private void scorePreloadSample() {
    outtake.closeClaw();
    //        sleep(150);
    //        outtake.closeClaw();
    arm.setArmPowers(1);
    sleep(600);
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.SCORE_HIGH_BUCKET_POSITION);
    strafeLeft(25);
    sleep(300);
    backward(12);
    sleep(200);
    outtake.openClaw();
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.PARK_POSITION);
    sleep(80);
    //        arm.setArmPowers(-0.9);
    //        sleep(200);
  }

  private void preSamplePark() {
    rotateRight(100);
    sleep(500);
    strafeLeft(300);
  }

  private void samplePark() {
    arm.setArmPowers(0.9);
    outtakeArm.goToPosition(Constants.OuttakeArmPosition.PARK_POSITION);
    forward(100);
    sleep(200);
    arm.setArmPowers(-0.9);
  }

  private void forward(long cm) {
    drive.driveRobotCentricFast(0.9, 0, 0);
    long time = cm * (2000 / 603);
    sleep(time);
    drive.stopMotors();
    sleep(100);
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
}
