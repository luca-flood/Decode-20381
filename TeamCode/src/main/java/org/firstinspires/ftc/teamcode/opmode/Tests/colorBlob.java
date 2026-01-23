package org.firstinspires.ftc.teamcode.opmode.Tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.opencv.core.Scalar;

import java.util.List;


@Disabled

@TeleOp(name = "color blob", group = "yes")
public class colorBlob extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        //camera intrinsics

        //res: 864 x 480
        double fx = 626.909;
        double fy = 626.909;

        double cx = 426.007;
        double cy = 236.834;


        /* res:
         */


        //define Color Ranges
        ColorRange greenRange = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar(15,  0,  0),     // Min (Y, Cr, Cb)
                new Scalar(255, 127, 170)   // Max (Y, Cr, Cb)
        );

        //custom PURPLE Range (YCrCb), these values are NOT final
        //purple is generally high in Red (Cr) and high in Blue (Cb)
        ColorRange purpleRange = new ColorRange(
                ColorSpace.YCrCb,
                new Scalar(32, 130, 140),  // Min (Y, Cr, Cb)
                new Scalar(255, 170, 200)   // Max (Y, Cr, Cb)
        );

        ColorBlobLocatorProcessor purpleProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(purpleRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false) //set to false for comp for faster processing
                .setBlurSize(6)
                .build();

        ColorBlobLocatorProcessor greenProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(greenRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(6)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(purpleProcessor)
                .addProcessor(greenProcessor)
                .setCameraResolution(new Size(864, 480)) // Your specified resolution
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .build();


        telemetry.setMsTransmissionInterval(25);


        while (opModeInInit()) {
            telemetry.addData("Camera Status", portal.getCameraState());
            telemetry.addData("Resolution", "864 x 480");
            telemetry.update();
        }


        waitForStart();


        while (opModeIsActive()) {

            Boolean blobDetected = checkForBlobs(portal, purpleProcessor, greenProcessor);

            // Telemetry Output
            if (blobDetected == null) {
                // Camera is not ready/streaming, so we cannot return false yet.
                telemetry.addData("Status", "WAITING FOR CAMERA...");
                telemetry.addData("Output", "N/A");
            } else {
                // Camera is streaming and we have a valid result
                telemetry.addData("Status", "STREAMING");
                telemetry.addData("Blob Detected:", blobDetected); // This is your True/False
            }

            // Debugging counts
            List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleProcessor.getBlobs();
            List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenProcessor.getBlobs();

            // Apply filter just for display count (logic is handled in function below)
            ColorBlobLocatorProcessor.Util.filterByArea(30, 20000, purpleBlobs);
            ColorBlobLocatorProcessor.Util.filterByArea(30, 20000, greenBlobs);

            telemetry.addData("Purple Count", purpleBlobs.size());
            telemetry.addData("Green Count", greenBlobs.size());

            telemetry.update();
        }
    }
    public Boolean checkForBlobs(VisionPortal portal,
                                  ColorBlobLocatorProcessor purpleProc,
                                  ColorBlobLocatorProcessor greenProc) {

        // 1. Verify Camera is actually STREAMING.
        // If it's OPENING, STARTING_STREAM, or ERROR, we cannot claim "False" yet.
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return null;
        }

        // 2. Get Blobs
        List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleProc.getBlobs();
        List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenProc.getBlobs();

        // 3. Filter out noise (Blobs smaller than 50 pixels area)
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, purpleBlobs);
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, greenBlobs);


        // 4. Check counts
        boolean seePurple = !purpleBlobs.isEmpty();
        boolean seeGreen = !greenBlobs.isEmpty();

        // 5. Return logic
        return seePurple || seeGreen;
    }
}
