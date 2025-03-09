package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.content.Context;
import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class AutonomousRecorder {
    private FileWriter fileWriter;
    private boolean isRecording = false;
    private boolean hasRecorded = false;

    private Telemetry telemetry;


    public AutonomousRecorder(Context context, Telemetry telemetry) {
        try {
            File directory = context.getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS);
            if (directory != null) {
                File file = new File(directory, "autonomous_data.csv");
                fileWriter = new FileWriter(file);
                // Write CSV header
                fileWriter.write("timestamp,fl,fr,bl,br,arm,claw\n");
            } else {
                throw new IOException("External files directory is null");
            }
            this.telemetry = telemetry;
        } catch (IOException e) {
            e.printStackTrace();
            isRecording = false;
        }
    }

    public void recordData(long timestamp, double fl, double fr, double bl, double br, double leftArm, double rightArm, double claw, double intakeLeft, double intakeRight, double leftFlipper, double rightFlipper) {
        if (!isRecording) return;
        try {
            String line = String.format("%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", timestamp, fl, fr, bl, br, leftArm, rightArm, claw, intakeLeft, intakeRight, leftFlipper, rightFlipper);
            fileWriter.write(line);

            telemetry.addData("Recorder, isRecording: ", isRecording);
            telemetry.addData("TimeStamp: ", timestamp);
            telemetry.addData("fLPower: ", fl);
            telemetry.addData("fRPower", fr);
            telemetry.addData("bLPower", bl);
            telemetry.addData("bRPower", br);
            telemetry.addData("leftArm", leftArm);
            telemetry.addData("rightArm", rightArm);
            telemetry.addData("claw", claw);
            telemetry.addData("intakeLeft", intakeLeft);
            telemetry.addData("intakeRight", intakeRight);
            telemetry.addData("leftFlipper", leftFlipper);
            telemetry.addData("rightFlipper", rightFlipper);
            telemetry.update();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean startRecording() {
        if (hasRecorded) return false;
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