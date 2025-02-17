package org.firstinspires.ftc.teamcode.autonomous;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class AutonomousRecorder {
    private FileWriter fileWriter;
    private boolean isRecording = false;
    private boolean hasRecorded = false;
    private long startTime;
    private long intervalMs;
    private long lastRecordTime = 0;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private Servo clawServo;

    public AutonomousRecorder(Context context, HardwareMap hardwareMap, long intervalMs) {
        this.intervalMs = intervalMs;
        try {
            File directory = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
            if (directory != null) {
                Log.d("AutonomousRecorder", "Saving CSV file to directory: " + directory.getAbsolutePath());

                File file = new File(directory, "motor_data.csv");
                fileWriter = new FileWriter(file);
                fileWriter.write("timestamp,front_left,front_right,back_left,back_right,arm,claw\n");
            } else {
                throw new IOException("External files directory is null");
            }
        } catch (IOException e) {
            e.printStackTrace();
            isRecording = false;
        }

        frontLeftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
        frontRightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
        backLeftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
        backRightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);
        armMotor = hardwareMap.get(DcMotor.class, Constants.ArmConstants.ARM_MOTOR_ID);
        clawServo = hardwareMap.get(Servo.class, Constants.ClawConstants.CLAW_SERVO_ID);
    }

    public void recordFrame() {
        if (!isRecording) return;

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastRecordTime >= intervalMs) {
            lastRecordTime = currentTime;
            long elapsedTime = currentTime - startTime;

            double fl = frontLeftMotor.getPower();
            double fr = frontRightMotor.getPower();
            double bl = backLeftMotor.getPower();
            double br = backRightMotor.getPower();
            double arm = armMotor.getPower();
            double claw = clawServo.getPosition();

            String line = String.format("%d,%f,%f,%f,%f,%f,%f\n",
                                        elapsedTime, fl, fr, bl, br, arm, claw);

            try {
                fileWriter.write(line);
                fileWriter.flush();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public boolean startRecording() {
        if (hasRecorded) {
            return false;
        }
        startTime = System.currentTimeMillis();
        hasRecorded = true;
        isRecording = true;
        return true;
    }

    public void stopRecording() {
        isRecording = false;
        try {
            fileWriter.flush();
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean getIsRecording() {
        return isRecording;
    }
}