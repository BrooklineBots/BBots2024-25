package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousRecorder;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousPlayer;

@TeleOp(name = "mainDrive")
public class RobotContainer extends OpMode {

    private Arm arm;
    private Claw claw;
    private MecanumDrive drive;

    private AutonomousRecorder recorder;
    private AutonomousPlayer player;

    private ElapsedTime recordingTimer;
    private boolean isRecording = false;
    private boolean isPlaying = false;

    @Override
    public void init() {
        arm = new Arm(hardwareMap, telemetry);
        claw = new Claw(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);

        recorder = new AutonomousRecorder(hardwareMap.appContext, hardwareMap, 20); // 100ms interval
        player = new AutonomousPlayer(hardwareMap);

        recordingTimer = new ElapsedTime();
    }

    public boolean isWithinTolerance(double targetValue, double currentValue, double tolerance) {
        return Math.abs(targetValue - currentValue) <= tolerance;
    }

    @Override
    public void loop() {
        telemetry.addData("Is Recording:", isRecording);
        telemetry.addData("Is Playing:", isPlaying);
        telemetry.update();

        if (isRecording && recordingTimer.seconds() >= 5.0) {
            recorder.stopRecording();
            isRecording = false;
            gamepad1.rumble(250);
        }

        if (isPlaying) {
            player.updatePlayback();
            if (!player.isPlaying()) {
                isPlaying = false;
                gamepad1.rumble(250);
            }
        }

        if (!isWithinTolerance(0, gamepad1.left_stick_y, 0.1) ||
                !isWithinTolerance(0, gamepad1.left_stick_x, 0.1) ||
                !isWithinTolerance(0, gamepad1.right_stick_x, 0.1)) {
            drive.driveFieldRelative(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        } else {
            drive.stopMotors();
        }

        if (!isWithinTolerance(0, gamepad2.left_stick_y, 0.1)) {
            arm.moveUp(gamepad2.left_stick_y);
        } else {
            arm.stop();
        }

        if (gamepad1.a) {
            claw.closeClaw();
        } else if (gamepad1.b) {
            claw.openClaw();
        }


        if (isRecording) {
            recorder.recordFrame();
        }

        if (gamepad1.x && !isRecording && !isPlaying) {
            if (recorder.startRecording()) {
                isRecording = true;
                recordingTimer.reset();
                gamepad1.rumble(250);
            }
        }

        if (gamepad1.y && !isRecording && !isPlaying) {
            player.loadRecording(hardwareMap.appContext);
            player.startPlayback();
            isPlaying = true;
            gamepad1.rumble(250);
        }
    }
}