package org.firstinspires.ftc.teamcode.autonomous;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class AutonomousPlayer {
    private static class MotorRecord {
        long timestamp;
        double frontLeft;
        double frontRight;
        double backLeft;
        double backRight;
        double arm;
        double claw;

        MotorRecord(long timestamp, double fl, double fr, double bl, double br, double arm, double claw) {
            this.timestamp = timestamp;
            this.frontLeft = fl;
            this.frontRight = fr;
            this.backLeft = bl;
            this.backRight = br;
            this.arm = arm;
            this.claw = claw;
        }
    }

    private List<MotorRecord> records = new ArrayList<>();
    private int currentIndex = 0;
    private long startTime;
    private boolean isPlaying = false;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private Servo clawServo;

    public AutonomousPlayer(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.FRONT_LEFT_MOTOR_ID);
        frontRightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.FRONT_RIGHT_MOTOR_ID);
        backLeftMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.BACK_LEFT_MOTOR_ID);
        backRightMotor = hardwareMap.get(DcMotor.class, Constants.DriveConstants.BACK_RIGHT_MOTOR_ID);
        armMotor = hardwareMap.get(DcMotor.class, Constants.ArmConstants.ARM_MOTOR_ID);
        clawServo = hardwareMap.get(Servo.class, Constants.ClawConstants.CLAW_SERVO_ID);
    }

    public void loadRecording(Context context) {
        records.clear();
        try {
            File directory = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
            File file = new File(directory, "motor_data.csv");
            Log.d("AutonomousPlayer", "Loading recording from: " + file.getAbsolutePath()); // Add logging

            if (!file.exists()) {
                Log.e("AutonomousPlayer", "CSV file does not exist");
                return;
            }

            BufferedReader reader = new BufferedReader(new FileReader(file));
            String line;
            boolean isHeader = true;
            while ((line = reader.readLine()) != null) {
                if (isHeader) {
                    isHeader = false;
                    continue;
                }
                String[] parts = line.split(",");
                if (parts.length != 7) continue;
                long timestamp = Long.parseLong(parts[0]);
                double fl = Double.parseDouble(parts[1]);
                double fr = Double.parseDouble(parts[2]);
                double bl = Double.parseDouble(parts[3]);
                double br = Double.parseDouble(parts[4]);
                double arm = Double.parseDouble(parts[5]);
                double claw = Double.parseDouble(parts[6]);
                records.add(new MotorRecord(timestamp, fl, fr, bl, br, arm, claw));
            }
            reader.close();
        } catch (IOException e) {
            Log.e("AutonomousPlayer", "Error loading recording", e);

            e.printStackTrace();
        }
    }
    public void startPlayback() {
        if (records.isEmpty()) return;

        startTime = System.currentTimeMillis();
        isPlaying = true;
        currentIndex = 0;
    }

    public void updatePlayback() {
        if (!isPlaying) return;

        long elapsed = System.currentTimeMillis() - startTime;

        // Process all records that have elapsed
        while (currentIndex < records.size()) {
            MotorRecord record = records.get(currentIndex);
            if (record.timestamp <= elapsed) {
                // Apply motor/servo values
                frontLeftMotor.setPower(record.frontLeft);
                frontRightMotor.setPower(record.frontRight);
                backLeftMotor.setPower(record.backLeft);
                backRightMotor.setPower(record.backRight);
                armMotor.setPower(record.arm);
                clawServo.setPosition(record.claw);
                currentIndex++;
            } else {
                break; // Wait for next timestamp
            }
        }

        // Stop playback only when all records are processed
        if (currentIndex >= records.size()) {
            stopPlayback();
        }
    }

    public void stopPlayback() {
        isPlaying = false;
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        armMotor.setPower(0);
    }

    public boolean isPlaying() {
        return isPlaying;
    }
}