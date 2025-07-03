package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.ClawArm;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalArm;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousRecorder;

@TeleOp(name = "robotDrive")
public class RobotContainer extends OpMode {

  private VerticalArm verticalArm;
  private Outtake outtake;
  private ClawArm clawArm;
  private MecanumDrive drive;
  private HorizontalExtension horizontal;
  //  private Limelight limelight;

  private boolean hasDefaulted = false;

  private AutonomousRecorder recorder;

  private ElapsedTime timer;

  private boolean isRedAlliance = true;

  private long recordingTimer;
  private final long startTimer = System.currentTimeMillis();
  private boolean isRecording = false;

  //delay variables
  private boolean highBucketTriggered = false;
  private long startTimeNs = -1;
  private boolean verticalMoved = false;
  private boolean clawArmMoved = false;

  public void setAlliance(boolean isRedAlliance) {
    this.isRedAlliance = isRedAlliance;
  }

  @Override
  public void init() {

    recorder = new AutonomousRecorder(hardwareMap.appContext, telemetry);

    verticalArm = new VerticalArm(hardwareMap, telemetry);
    outtake = new Outtake(hardwareMap, telemetry);
    clawArm = new ClawArm(hardwareMap, telemetry);
    drive = new MecanumDrive(hardwareMap, telemetry);
    horizontal = new HorizontalExtension(hardwareMap, telemetry);
    //    limelight = new Limelight(hardwareMap, telemetry, isRedAlliance);
    //    limelight.start();
    recordingTimer = System.currentTimeMillis() - startTimer;
  }

  public boolean isWithinTolerance(
      final double targetValue, final double currentValue, final double tolerance) {
    return Math.abs(targetValue - currentValue) <= tolerance;
  }

  public void toggleAlliance() {
    this.isRedAlliance = !isRedAlliance;
  }

  @Override
  public void loop() {
    //default vert arm and clawArm
    if(!hasDefaulted){
      verticalArm.goToPosition(Constants.ArmPosition.STOWED);
      clawArm.moveToTransfer();
      hasDefaulted = true;
    }

    /*
    if (gamepad1.x) {
      if (recorder.startRecording()) {
        isRecording = true;
        recordingTimer = System.currentTimeMillis();
        gamepad1.rumble(250);
      }
    }

    if (isRecording) {
      final double[] drivePowers = drive.getPowers();
      final double[] armPower = verticalArm.getArmPowers();
      final double[] wheelPower = {0, 0}; // intake.getWheelPowers();
      final double clawPosition = claw.getClawPosition();
      final double[] flipperPosition = intake.getFlipperPos();
      recordingTimer = System.currentTimeMillis() - startTimer;

      recorder.recordData(
          recordingTimer,
          drivePowers[0], // Front Left
          drivePowers[1], // Front Right
          drivePowers[2], // Back Left
          drivePowers[3], // Back Right
          armPower[0], // Left Arm
          armPower[1], // Right Arm
          clawPosition, // Claw
          wheelPower[0], // Intake Left
          wheelPower[1], // Intake Right
          flipperPosition[0], // leftFlipper
          flipperPosition[1] // rightFlipper
          );
      telemetry.update();
    } else {
      //            telemetry.addData("Container, isRecording: ", isRecording);
      //            telemetry.update();
    }


    if (isRecording && ((double) recordingTimer / 1000) >= 30.0) {
      recorder.stopRecording();
      isRecording = false;
      gamepad1.rumble(250);
    }

     */ //recorder

    //field relative drive
    final double forward = -gamepad1.left_stick_y;
    final double right = gamepad1.left_stick_x;
    final double rotate = gamepad1.right_stick_x;

    if (!isWithinTolerance(0, gamepad1.left_stick_y, 0.1)
        || !isWithinTolerance(0, gamepad1.left_stick_x, 0.1)
        || !isWithinTolerance(0, gamepad1.right_stick_x, 0.1)) {
      drive.driveFieldRelative(forward, right, rotate);
    } else {
      drive.stop();
    }

    if (gamepad2.b) {
      drive.resetYaw();
    }

    /*limelight
    final boolean driveSticksActive =
        !isWithinTolerance(0, gamepad1.left_stick_y, 0.05)
            || !isWithinTolerance(0, gamepad1.left_stick_x, 0.05)
            || !isWithinTolerance(0, gamepad1.right_stick_x, 0.05);

    if (driveSticksActive && gamepad1.right_trigger > 0.25 && limelight.hasTarget()) {
      final double rotCmd = limelight.getRotationCorrection();
      drive.driveFieldRelative(forwardInput, strafeInput, rotCmd);
    } else if (driveSticksActive) {
      drive.driveFieldRelative(forwardInput, strafeInput, rotateInput);
    } else {
      drive.stop();
    }

     */ //limelight

    telemetry.addData("Claw Arm Moved: ", clawArmMoved);
    telemetry.addData("Vertical Arm Moved: ", verticalMoved);
    telemetry.addData("Start Time Nano Seconds: ", startTimeNs);


    //scoring
    //testing TODO: Add claw open and close


    //emergency vertical arm control:
    if(!isWithinTolerance(0, gamepad2.left_trigger, 0.1)){

      if(!isWithinTolerance(0, gamepad2.left_stick_y, 0.1)){
        verticalArm.setModeRunWithEncoder();
        double power = gamepad2.left_stick_x;
        //limit power
        power *= 0.5;
        verticalArm.setArmPowers(power);

      }
      else{
        verticalArm.stop();
      }
    }
    else{
      if(gamepad2.dpad_up && !highBucketTriggered){
        highBucketTriggered = true;
        startTimeNs = System.nanoTime();
        verticalArm.goToPosition(Constants.ArmPosition.SCORE_HIGH_BUCKET);
        verticalMoved = true;
      } else if (gamepad2.dpad_down){
        verticalArm.goToPosition(Constants.ArmPosition.STOWED);
        clawArm.goToPosition(Constants.ClawArmPosition.SCORE_LOW_BUCKET_POSITION);
      } else if(gamepad2.dpad_right){
        verticalArm.goToPosition(Constants.ArmPosition.STOWED);
        clawArm.goToPosition(Constants.ClawArmPosition.SCORE_HIGH_BAR_POSITION);
      } else if(gamepad2.dpad_left){
        verticalArm.goToPosition(Constants.ArmPosition.STOWED);
        clawArm.goToPosition(Constants.ClawArmPosition.TRANSFER_POSITION);
      }
    }
    
    //CLAW HIGH BUCKET DELAY
    if (highBucketTriggered) {
      long currentTime = System.nanoTime();
      double elapsedTime = (currentTime - startTimeNs) / 1e9;
      telemetry.addData("Elapsed time", elapsedTime);

      if (verticalMoved && !clawArmMoved && elapsedTime >= Constants.ClawArmConstants.CLAW_ARM_DELAY_BUCKET) {
        clawArm.goToPosition(Constants.ClawArmPosition.SCORE_HIGH_BUCKET_POSITION);
        clawArmMoved = true;
        highBucketTriggered = false;
        verticalMoved = false;
      }
    }



//    if (gamepad2.dpad_up) { //SCORE HIGH BUCKET
//      verticalArm.goToPosition(Constants.ArmPosition.SCORE_HIGH_BUCKET);
//      clawArm.scoreHighBucket();
//      outtake.openClaw();
//
//    } else if (gamepad2.dpad_down) {
//      verticalArm.goToPosition(Constants.ArmPosition.STOWED);
//      clawArm.scoreLowBucket();
//      outtake.openClaw();
//
//    } else if (gamepad2.dpad_right) {
//      verticalArm.goToPosition(Constants.ArmPosition.GO_TO_HIGH_BAR);
//      clawArm.scoreHighBar();
//      if(outtake.getGoalPosition() != Constants.OuttakePosition.CLOSE_POSITION){
//        outtake.closeClaw();
//      }
//    } else if(gamepad2.dpad_left){
//      verticalArm.goToPosition(Constants.ArmPosition.SCORE_HIGH_BAR);
//      outtake.openClaw();
//    }

    //basic outtake control
    if (gamepad1.left_bumper) {
        outtake.closeClaw();
      } else if (gamepad1.right_bumper) {
        outtake.openClaw();
    }

    //horizontal extension
    if (!isWithinTolerance(0, gamepad2.right_trigger, 0.1)) {
      horizontal.extend();
    } else if (!isWithinTolerance(0, gamepad2.left_trigger, 0.1)) {
      horizontal.retract();
    } else{
      horizontal.stopServos();
    }

    //toggle alliance
    if (gamepad2.a) {
      toggleAlliance();
    }

    //telemetry
    telemetry.addData("alliance:", isRedAlliance);
    telemetry.addData("Left Arm Position: ", verticalArm.getCurrentPosition()[0]);
    telemetry.addData("Right Arm Position: ", verticalArm.getCurrentPosition()[1]);
    telemetry.update();
  }
}
