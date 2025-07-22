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

        if(opModeIsActive()){

            outtakeArm.goToPosition(Constants.OuttakeArmPosition.AUTONOMOUS_POSITION);
            intake.goToPositionFlip(Constants.IntakePosition.FLIP_TRANSFER_POSITION);
            if(!touchSensor.isPressed()){
                horizontalExtension.retract();
            } else{
                horizontalExtension.stopServos();
            }

            //score first specimen
            backward(20);
            strafeRight(54);
            backward(38);
            goToHighBar(65);
            scoreSpecimen(35);
            sleep(1000);

            //score second specimen
            forward(10);


            rotateLeft(1850);
            strafeRight(90);
            forward(120);
            strafeRight(27);
            rotateLeft(50);
            backward(165);
            forward(40);
            driveWait(3);
            moveToPickup(29);
            rotateRight(10);
            backward(45);
            sleep(100);
            grabSpecimen(29);
            sleep(100);
            forward(40);
            rotateLeft(1900);
            strafeRight(100);
            rotateLeft(50);
            goToHighBar(54);
            backward(17);
            scoreSpecimen(30);

            //backward(30);




        }
    }

    private void forward(long cm){
        drive.driveRobotCentric(0.5, 0, 0);
        long time = cm * (500/45);
        sleep(time);
        drive.stopMotors();
    }
    private void backward(long cm){
        drive.driveRobotCentric(-0.5, 0, 0);
        long time = cm * (500/45);
        sleep(time);
        drive.stopMotors();
    }
    private void strafeRight(long cm){
        drive.driveRobotCentric(0, 0.5, 0);
        long time = cm * (1000/37);
        sleep(time);
        drive.stopMotors();
    }
    private void strafeLeft(long cm){
        drive.driveRobotCentric(0, -0.5, 0);
        long time = cm * (1000/37);
        sleep(time);
        drive.stopMotors();
    }
    private void rotateRight(long time){
        drive.driveRobotCentric(0, 0, 0.5);
        sleep(time);
        drive.stopMotors();
    }
    private void rotateLeft(long time){
        drive.driveRobotCentric(0, 0, -0.5);
        sleep(time);
        drive.stopMotors();
    }
    private void driveWait(long seconds){
        drive.stopMotors();
        long time = seconds * 1000;
        sleep(time);
    }
    private void grabSpecimen(long cm){
        outtake.closeClaw();
        sleep(1000);
        outtake.closeClaw();
        long time = cm * (225/8);
        arm.setArmPowers(0.5);
        sleep(time);
        arm.setArmPowers(0);
    }
    private void moveToPickup(long cm){
        outtakeArm.goToPosition(Constants.OuttakeArmPosition.PICKUP_POSITION);
        sleep(2000);
        outtake.openClaw();
        long time = cm * (225/8);
        arm.setArmPowers(-0.5);
        sleep(time);
        arm.setArmPowers(0);
    }

    private void goToHighBar(long cm){
        outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
        sleep(500);
        outtake.closeClaw();
        sleep(100);
        outtake.closeClaw();
        long time = cm * (225/8);
        arm.setArmPowers(0.5);
        sleep(time);
        arm.setArmPowers(0);
    }

    private void scoreSpecimen(long cm){
        outtake.closeClaw();
        long time = cm * (225/8);
        outtakeArm.goToPosition(Constants.OuttakeArmPosition.GO_TO_HIGH_BAR_POSITION);
        arm.setArmPowers(-0.5);
        sleep(time);
        arm.setArmPowers(0);
        sleep(100);
        outtake.openClaw();
        sleep(500);

    }
}
