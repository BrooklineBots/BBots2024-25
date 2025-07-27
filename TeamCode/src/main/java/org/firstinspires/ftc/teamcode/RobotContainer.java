package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Subsystems.HomingSensor;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.Subsystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalArm;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousRecorder;

@TeleOp(name = "robotDrive")
public class RobotContainer extends OpMode {

  private VerticalArm verticalArm;
  private Outtake outtake;
  private OuttakeArm outtakeArm;
  private MecanumDrive drive;
  private HorizontalExtension horizontal;
  private Intake intake;
  private HomingSensor homingSensor;
  //  private Limelight limelight;

  private double initialHeading;

  private AutonomousRecorder recorder;

  private ElapsedTime timer;

  private boolean isRedAlliance = true;

  private boolean hasDefaulted = false;
  private long recordingTimer;
  private final long startTimer = System.currentTimeMillis();
  private final boolean isRecording = false;

  // delay variables
  private boolean transferTriggered = false;
  private boolean intakeMovedToTransfer = false;
  private boolean outtakeMovedToTransfer = false;
  private final long startTimeNs = -1;

  public void setAlliance(final boolean isRedAlliance) {
    this.isRedAlliance = isRedAlliance;
  }

  @Override
  public void init() {

    recorder = new AutonomousRecorder(hardwareMap.appContext, telemetry);

    verticalArm = new VerticalArm(hardwareMap, telemetry);
    outtake = new Outtake(hardwareMap, telemetry);
    outtakeArm = new OuttakeArm(hardwareMap, telemetry);
    drive = new MecanumDrive(hardwareMap, telemetry);
    horizontal = new HorizontalExtension(hardwareMap, telemetry);
    homingSensor = new HomingSensor(hardwareMap, telemetry);
    intake = new Intake(hardwareMap, telemetry);

    recordingTimer = System.currentTimeMillis() - startTimer;
  }

  public boolean isWithinTolerance(
      final double targetValue, final double currentValue, final double tolerance) {
    return Math.abs(targetValue - currentValue) <= tolerance;
  }

  public void toggleAlliance() {
    this.isRedAlliance = !isRedAlliance;
  }

  //  public void transferSamples(){
  //    if(isWithinTolerance(verticalArm.getCurrentPosition()[1],
  // Constants.ArmPosition.STOWED.encoderTicks, 100)){
  //      outtake.closeClaw();
  //
  //    }
  //  }

  @Override
  public void loop() {
    if (!hasDefaulted) {
      intake.goToPositionFlip(Constants.IntakePosition.FLIP_TRANSFER_POSITION);
      hasDefaulted = true;
    }

    // field relative drive
    final double forward = -gamepad1.left_stick_y;
    final double right = gamepad1.left_stick_x;
    final double rotate = gamepad1.right_stick_x;

    if (!isWithinTolerance(0, gamepad1.left_stick_y, 0.1)
        || !isWithinTolerance(0, gamepad1.left_stick_x, 0.1)
        || !isWithinTolerance(0, gamepad1.right_stick_x, 0.1)) {
      drive.driveRobotCentricSlow(forward, right, rotate);
    } else {
      drive.stopMotors();
    }

    if (gamepad2.b) {
      initialHeading = drive.getBotHeading();
      drive.resetYaw();
      drive.setFieldHeadingOffset(initialHeading + Math.PI);
    }

    telemetry.addData("Start Time Nano Seconds: ", startTimeNs);
    telemetry.addData(
        "Arm Height IN: ",
        Utils.ticksToInches(
            verticalArm.getCurrentPosition()[0],
            Constants.ArmConstants.ARM_BELT_LENGTH,
            1)); // No gear ratio

    // scoring
    // testing TODO: Add claw open and close

    // emergency vertical arm control:

    if (!isWithinTolerance(0, gamepad2.left_stick_y, 0.1)) {
      verticalArm.setModeRunWithoutEncoder();
      final double power = -gamepad2.left_stick_y;
      // limit power
      // power *= 0.5;
      verticalArm.setArmPowers(power);

    } else {
      verticalArm.stop();
    }

    if (gamepad2.dpad_up) {
      outtakeArm.goToPosition(Constants.OuttakeArmPosition.SCORE_HIGH_BUCKET_POSITION);
    } else if (gamepad2.dpad_right) {
      outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
    } else if (gamepad2.dpad_left) {
      outtakeArm.goToPosition(Constants.OuttakeArmPosition.TRANSFER_POSITION);
    } else if (gamepad2.dpad_down) {
      outtakeArm.goToPosition(Constants.OuttakeArmPosition.PICKUP_POSITION);
    }

    // CLAW HIGH BUCKET DELAY

    if (transferTriggered) {
      final long currentTime = System.nanoTime();
      final double elapsedTime = (currentTime - startTimeNs) / 1e9;

      if (intakeMovedToTransfer
          && !outtakeMovedToTransfer
          && elapsedTime >= Constants.OuttakeArmConstants.OUTTAKE_DELAY_TRANSFER) {
        outtakeArm.goToPosition(Constants.OuttakeArmPosition.TRANSFER_POSITION);
        outtakeMovedToTransfer = true;
        transferTriggered = false;
        intakeMovedToTransfer = false;
      }
    }

    // intake control on gamepad1
    if (gamepad1.y) {
      intake.goToPositionFlip(Constants.IntakePosition.FLIP_TRANSFER_POSITION);
    }
    if (gamepad1.a) {

      intake.goToPositionFlip(Constants.IntakePosition.FLIP_PICKUP_POSITION);
    }
    if (gamepad2.x) {
      intake.goToPositionFlip(Constants.IntakePosition.FLIP_HOVER_POSITION);
    }

    if (gamepad1.x) {
      intake.openClaw();
    }
    if (gamepad1.b) {

      intake.closeClaw();
    }

    if (!isWithinTolerance(0, gamepad2.right_stick_x, 0.1)) {
      intake.rotateIntake(gamepad2.right_stick_x);
    } else {
      intake.stopRotate();
    }

    // basic outtake control
    if (gamepad2.left_bumper) {
      outtake.closeClaw();
    } else if (gamepad2.right_bumper) {
      outtake.openClaw();
    }

    // horizontal extension
    if (gamepad1.right_bumper) {
      horizontal.extend();
    } else if (gamepad1.left_bumper && !homingSensor.isPressed()) {
      horizontal.retract();
    } else if (gamepad2.y) {
      while (!homingSensor.isPressed()) {
        horizontal.home();
      }
      horizontal.stopServos();
    } else {
      horizontal.stopServos();
    }

    if (homingSensor.isPressed()) {
      horizontal.stopServos();
    }

    // toggle alliance
    if (gamepad2.a) {
      toggleAlliance();
    }

    // telemetry
    telemetry.addData("alliance:", isRedAlliance);
    telemetry.addData("Left Arm Position: ", verticalArm.getCurrentPosition()[0]);
    telemetry.addData("Right Arm Position: ", verticalArm.getCurrentPosition()[1]);
    telemetry.addData("touchSensorPressed: ", homingSensor.isPressed());
    telemetry.update();
  }
}
