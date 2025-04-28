package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp
public class WebcamExample2 extends LinearOpMode {

  OpenCvWebcam webcam;

  // purple: 150, 110, 160
  // blue: 100, 200, 170
  // orange: 30, 100, 140

  @Override
  public void runOpMode() {
    final int cameraMonitorViewId =
        hardwareMap
            .appContext
            .getResources()
            .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam =
        OpenCvCameraFactory.getInstance()
            .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    webcam.setPipeline(new SamplePipeline());
    webcam.setMillisecondsPermissionTimeout(
        2500); // Timeout for obtaining permission is configurable. Set before opening.
    webcam.openCameraDeviceAsync(
        new OpenCvCamera.AsyncCameraOpenListener() {
          @Override
          public void onOpened() {
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
          }

          @Override
          public void onError(final int errorCode) {
            /*
             * This will be called if the camera could not be opened
             */
          }
        });
    telemetry.addLine("Waiting for start");
    telemetry.update();
    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Frame Count", webcam.getFrameCount());
      telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
      telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
      telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
      telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
      telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
      telemetry.update();
    }
  }

  class SamplePipeline extends OpenCvPipeline {
    boolean viewportPaused;

    @Override
    public Mat processFrame(final Mat input) {
      final Mat mat = new Mat();
      Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

      int hsum = 0;
      int ssum = 0;
      int vsum = 0;

      for (int i = 0; i < mat.rows(); i++) {
        for (int j = 0; j < mat.cols(); j++) {
          final double[] pixel = mat.get(i, j);
          hsum += pixel[0];
          ssum += pixel[1];
          vsum += pixel[2];
        }
      }
      final int hval = hsum / (mat.rows() * mat.cols());
      final int sval = ssum / (mat.rows() * mat.cols());
      final int vval = vsum / (mat.rows() * mat.cols());

      telemetry.addData("H:", hval);
      telemetry.addData("S:", sval);
      telemetry.addData("V:", vval);
      telemetry.update();

      return input;
    }

    @Override
    public void onViewportTapped() {
      viewportPaused = !viewportPaused;

      if (viewportPaused) {
        webcam.pauseViewport();
      } else {
        webcam.resumeViewport();
      }
    }
  }
}
