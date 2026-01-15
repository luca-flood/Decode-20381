package org.firstinspires.ftc.teamcode.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Disabled

@TeleOp(name = "apriltagTest", group = "Linear Opmode")
public class apriltagTest extends LinearOpMode {



   /* values for 1920x1080
    public double fx = 1734.816;
    public double fy = 1300.112;
    public double cx = 1206.435;
    public double cy = 498.389;
*/

/* values for 640x480
   public double fx = 578.272
   public double fy = 578.272
   public double cx = 402.145
   public double  cy = 221.506
 */


    // values for 1280x720
    public double fx = 1156.544;
    public double fy = 867.408;
    public double cx = 804.290;
    public double cy = 332.259;
    public boolean calibratedCamera;
    public AprilTagProcessor aprilTagProcessor;



    @Override
    public void runOpMode() throws InterruptedException {

        if (calibratedCamera) {
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        } else {
            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(fx, fy, cx, cy)
                    .build();
        }

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Camera"))
                .addProcessor(aprilTagProcessor)
                .build();

        Servo axon = hardwareMap.get(Servo.class, "axon");

        waitForStart();

        while (opModeIsActive()) {
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                int id = detection.id;
                double x = detection.ftcPose.x;
                double y = detection.ftcPose.y;
                double z = detection.ftcPose.z;
                double pitch = detection.ftcPose.pitch;
                double yaw = detection.ftcPose.yaw;
                double roll = detection.ftcPose.roll;
                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing;

                if (gamepad1.a) {
                    telemetry.addLine(String.valueOf(detection.id));
                    telemetry.addLine(String.valueOf(detection.ftcPose.x));
                    telemetry.addLine(String.valueOf(detection.ftcPose.y));
                    telemetry.addLine(String.valueOf(detection.ftcPose.z));
                    telemetry.addLine(String.valueOf(detection.ftcPose.pitch));
                    telemetry.addLine(String.valueOf(detection.ftcPose.yaw));
                    telemetry.addLine(String.valueOf(detection.ftcPose.roll));
                    telemetry.addLine(String.valueOf(detection.ftcPose.range));
                    telemetry.addLine(String.valueOf(detection.ftcPose.bearing));
                    telemetry.update();
                }

            }
            if (gamepad1.x){
                axon.setPosition(1);
            }
            if (gamepad1.y){
                axon.setPosition(0);
            }
            if (gamepad1.b){
                axon.setPosition(0.5);
            }

        }
    }
}