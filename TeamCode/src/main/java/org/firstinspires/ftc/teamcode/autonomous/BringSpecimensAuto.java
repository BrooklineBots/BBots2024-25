package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.HomingSensor;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalExtension;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@Autonomous(name = "BringSpecimens", group = "Autonomous")
public class BringSpecimensAuto extends LinearOpMode {
  private MecanumDrive drive;
  private HorizontalExtension horizontalExtension;
  private Intake intake;
  private HomingSensor touchSensor;

  @Override
  public void runOpMode() throws InterruptedException {
    drive = new MecanumDrive(hardwareMap, telemetry);
    horizontalExtension = new HorizontalExtension(hardwareMap, telemetry);
    intake = new Intake(hardwareMap, telemetry);
    touchSensor = new HomingSensor(hardwareMap, telemetry);

    waitForStart();

    if (opModeIsActive()) {
      intake.goToPositionFlip(Constants.IntakePosition.FLIP_TRANSFER_POSITION);
      if (!touchSensor.isPressed()) {
        horizontalExtension.retract();
      } else {
        horizontalExtension.stopServos();
      }
      forward(45);
      strafeRight(45);
      forward(110);
      strafeRight(27);
      backward(135);
      forward(10);
      strafeLeft(10);
      forward(135);
      strafeRight(35);
      rotateRight(50);
      backward(135);
      forward(145);
      strafeRight(19);
      backward(135);
    }
  }

  private void forward(long cm) {
    drive.driveRobotCentricSlow(0.5, 0, 0);
    long time = cm * (500 / 45);
    sleep(time);
    drive.stopMotors();
  }

  private void backward(long cm) {
    drive.driveRobotCentricSlow(-0.5, 0, 0);
    long time = cm * (500 / 45);
    sleep(time);
    drive.stopMotors();
  }

  private void strafeRight(long cm) {
    drive.driveRobotCentricSlow(0, 0.5, 0);
    long time = cm * (1000 / 37);
    sleep(time);
    drive.stopMotors();
  }

  private void strafeLeft(long cm) {
    drive.driveRobotCentricSlow(0, -0.5, 0);
    long time = cm * (1000 / 37);
    sleep(time);
    drive.stopMotors();
  }

  private void rotateRight(long time) {
    drive.driveRobotCentricSlow(0, 0, 0.5);
    sleep(time);
    drive.stopMotors();
  }

  private void rotateLeft(long time) {
    drive.driveRobotCentricSlow(0, 0, -0.5);
    sleep(time);
    drive.stopMotors();
  }
}
