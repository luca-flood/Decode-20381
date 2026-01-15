/*package org.firstinspires.ftc.teamcode.opmode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
@Disabled
@TeleOp(name="Driver Control BLUE", group="Linear Opmode")


public class DriverBlue extends LinearOpMode {



    private OpenCvCamera controlHubCam;
    private static final int CAMERA_WIDTH = 1920;
    private static final int CAMERA_HEIGHT = 1080;
    private Servo LiftServo;
    private double angle = 0;



    @Override
    public void runOpMode() throws InterruptedException {
// Declaration and mapping of lift motors.
        DistanceSensor sensorDistance;

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 60);


// Declaration and mapping of drive motors.
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");

        //motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Extend and rotate
        DcMotor liftRotateMotorRight = hardwareMap.dcMotor.get("liftRotateMotorRight");
        DcMotor liftRotateMotorLeft = hardwareMap.dcMotor.get("liftRotateMotorLeft");
        DcMotor liftExtendMotorRight = hardwareMap.dcMotor.get("liftExtendMotorRight");
        DcMotor liftExtendMotorLeft = hardwareMap.dcMotor.get("liftExtendMotorLeft");


        DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "muhaddas i love boys");

        // Reverse the right side motors
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);


        liftRotateMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        liftRotateMotorRight.setDirection(DcMotor.Direction.FORWARD);
        liftExtendMotorRight.setDirection(DcMotor.Direction.REVERSE);
        liftExtendMotorLeft.setDirection(DcMotor.Direction.FORWARD);

        //liftRotateMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftRotateMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtendMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtendMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftRotateMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRotateMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftExtendMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftExtendMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        liftExtendMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//Set target position and power of lift motors

        liftRotateMotorRight.setTargetPosition(500);
        liftRotateMotorLeft.setTargetPosition(500);
        liftExtendMotorRight.setTargetPosition(0);
        liftExtendMotorLeft.setTargetPosition(0);

        liftRotateMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftRotateMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftRotateMotorRight.setPower(1);
        liftRotateMotorLeft.setPower(1);
        liftExtendMotorRight.setPower(0.75);
        liftExtendMotorLeft.setPower(0.75);


        Servo LeftClawServo = hardwareMap.get(Servo.class, "LeftClawServo");
        Servo RightClawServo = hardwareMap.get(Servo.class, "RightClawServo");
        Servo ClawRotateServo = hardwareMap.get(Servo.class, "ClawRotateServo");

        Servo rightWrist = hardwareMap.get(Servo.class, "rightWrist");
        Servo leftWrist = hardwareMap.get(Servo.class, "leftWrist");

        Servo LeftBackServo = hardwareMap.get(Servo.class, "backLeft");
        Servo RightBackServo = hardwareMap.get(Servo.class, "backRight");



        boolean override = false;
        boolean override1 = false;
        boolean aPress1 = false;
        boolean aPress2 = false;

        boolean bPress1 = false;
        boolean bPress2 = false;


        waitForStart();

        liftRotateMotorRight.setPower(1);
        liftRotateMotorLeft.setPower(1);
        liftExtendMotorRight.setPower(1);
        liftExtendMotorLeft.setPower(1);

        boolean isClosed = false;

        double position = 0;
        double right1 = 0;
        double left1 = 0;

        int rightExtendPosition = 0;
        int leftExtendPosition = 0;
        int rotatePosition = 0;

        ClawRotateServo.setPosition(0.5);
        RightClawServo.setPosition(0.0);
        LeftClawServo.setPosition(1);


        rightWrist.setPosition(1);
        leftWrist.setPosition(1);

        boolean left = false;
        boolean right = false;

        boolean highlevel = false;
        boolean proxyFlag = false;

        boolean backClawClosed = true;
        boolean frontClawClosed = true;


        int position2 = 0;
        double angle2 = 0;
        final double TRIGGER_THRESHOLD = 0.75;

        while (opModeIsActive()) {


            telemetry.addLine("left");
            telemetry.update();


//Drive Motor Code!
//Not to be messed with under any circumstances.
            liftRotateMotorRight.setPower(1);
            liftRotateMotorLeft.setPower(1);
            liftExtendMotorRight.setPower(1);
            liftExtendMotorLeft.setPower(1);

            ElapsedTime runtime = new ElapsedTime();
            double y = gamepad1.left_stick_y;
            double rx = -gamepad1.right_stick_x * 1.1;
            double x = -gamepad1.left_stick_x;

// Stupid Code
            double y2 = gamepad2.left_stick_y;
            double rx2 = -gamepad2.right_stick_x * 1.1;
            double x2 = -gamepad2.left_stick_x;

            telemetry.addLine("left stick y" + y);
            telemetry.addLine("left stick x" + x);
            telemetry.addLine("right stick x" + rx);
// telemetry.addLine("servo pos: " + rightliftServo.getPosition());


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double denominator2 = Math.max(Math.abs(y2) + Math.abs(x2) + Math.abs(rx2), 1);
            double frontLeftPower2 = (y2 + x2 + rx2) / denominator2;
            double backLeftPower2 = (y2 - x2 + rx2) / denominator2;
            double frontRightPower2 = (y2 - x2 - rx2) / denominator2;
            double backRightPower2 = (y2 + x2 - rx2) / denominator2;


            if (gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_x >= 0.1 || gamepad1.left_stick_y <= -0.1 || gamepad1.left_stick_x <= -0.1 || gamepad1.right_stick_y >= 0.2 || gamepad1.right_stick_x >= 0.2 || gamepad1.right_stick_y <= -0.2 || gamepad1.right_stick_x <= -0.2) {
                motorFrontLeft.setPower(frontLeftPower * 0.85);
                motorBackLeft.setPower(backLeftPower * 0.85);
                motorFrontRight.setPower(frontRightPower * 0.85);
                motorBackRight.setPower(backRightPower * 0.85);
            } else if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1) {
                motorFrontLeft.setPower(frontLeftPower2 * 0.3);
                motorBackLeft.setPower(backLeftPower2 * 0.3);
                motorFrontRight.setPower(frontRightPower2 * 0.3);
                motorBackRight.setPower(backRightPower2 * 0.3);
            } else {
                motorFrontLeft.setPower(0);
                motorBackLeft.setPower(0);
                motorFrontRight.setPower(0);
                motorBackRight.setPower(0);
            }

            //telemetry.addData("Current Angle", angle);// Ensure it's added initially

            if(gamepad1.dpad_up && override == false){
                leftWrist.setPosition(0.5);
                rightWrist.setPosition(0.5);
                override = true;
                sleep(500);
            }else if(gamepad1.dpad_up && override == true){
                leftWrist.setPosition(0.2);
                rightWrist.setPosition(0.2);
                override = false;
                sleep(500);

            }

            if(gamepad2.dpad_up && override1 == false){
                leftWrist.setPosition(0.5);
                rightWrist.setPosition(0.5);
                override1 = true;
                sleep(500);
            }else if(gamepad2.dpad_up && override1 == true){
                leftWrist.setPosition(0.2);
                rightWrist.setPosition(0.2);
                override1 = false;
                sleep(500);

            }



            if(gamepad1.dpad_down){
                leftWrist.setPosition(1);
                rightWrist.setPosition(1);
                sleep(500);
            }

            if(gamepad2.dpad_down){
                leftWrist.setPosition(1);
                rightWrist.setPosition(1);
                sleep(500);
            }






//Lift Motor Code
// Find starting height for lift and find power at that point
// Find final height and power and map that to the claw orientation (position 0 < x < 1)

            double liftPowerDown = gamepad1.left_trigger;
            double liftPowerUp = gamepad1.right_trigger;

            double liftPowerDown2 = gamepad2.left_trigger;
            double liftPowerUp2 = gamepad2.right_trigger;

            //Claw code - Proximity sensors

//            if (gamepad1.dpad_left || gamepad2.dpad_right){
//                if (ClawRotateServo.getPosition() > 0.1){
//                    ClawRotateServo.setPosition(ClawRotateServo.getPosition() - 0.1);
//                }
//            }
//
//            if (gamepad1.dpad_right || gamepad2.dpad_right){
//                if (ClawRotateServo.getPosition() < 0.9){
//                    ClawRotateServo.setPosition(ClawRotateServo.getPosition() + 0.1);
//                }
//            }


            //Lift rotation

            if (gamepad1.right_trigger >= 0.3) {
                liftRotateMotorRight.setPower(1);
                liftRotateMotorLeft.setPower(1);
                //Don’t know yet
                rotatePosition = (int) (liftRotateMotorRight.getCurrentPosition() + (70 * gamepad1.right_trigger));
                sleep(10);
                liftRotateMotorRight.setTargetPosition(rotatePosition);
                liftRotateMotorLeft.setTargetPosition(rotatePosition);
            } else if (gamepad2.right_trigger >= 0.3) {
                liftRotateMotorRight.setPower(1);
                liftRotateMotorLeft.setPower(1);
                rotatePosition = (int) (liftRotateMotorRight.getCurrentPosition() + (45 * gamepad2.right_trigger));
                liftRotateMotorRight.setTargetPosition(rotatePosition);
                liftRotateMotorLeft.setTargetPosition(rotatePosition);
            }


            if (gamepad1.left_trigger >= 0.3) {
                liftRotateMotorRight.setPower(1);
                liftRotateMotorLeft.setPower(1);
                rotatePosition = (int) (liftRotateMotorRight.getCurrentPosition() - (45 * gamepad1.left_trigger));
                sleep(10);
                liftRotateMotorRight.setTargetPosition(rotatePosition);
                liftRotateMotorLeft.setTargetPosition(rotatePosition);
            } else if (gamepad2.left_trigger >= 0.3) {
                liftRotateMotorRight.setPower(1);
                liftRotateMotorLeft.setPower(1);
                rotatePosition = (int) (liftRotateMotorRight.getCurrentPosition() - (45 * gamepad2.left_trigger));
                liftRotateMotorRight.setTargetPosition(rotatePosition);
                liftRotateMotorLeft.setTargetPosition(rotatePosition);
            }


            //Extension

            if (gamepad1.right_bumper) {
                if (rightExtendPosition > -4500) {
                    //liftExtendMotorRight.setPower(0.85);
                    //liftExtendMotorLeft.setPower(0.85);
                    liftExtendMotorRight.setPower(1);
                    liftExtendMotorLeft.setPower(1);
                    rightExtendPosition = liftExtendMotorRight.getCurrentPosition() - 750;
                    leftExtendPosition = liftExtendMotorLeft.getCurrentPosition() - 750;
                    sleep(1);
                    liftExtendMotorRight.setTargetPosition(rightExtendPosition);
                    liftExtendMotorLeft.setTargetPosition(leftExtendPosition);
                }
            } else if (gamepad2.right_bumper) {
                if (rightExtendPosition > -4500) {
                    //liftExtendMotorRight.setPower(0.85);
                    //liftExtendMotorLeft.setPower(0.85);
                    liftExtendMotorRight.setPower(1);
                    liftExtendMotorLeft.setPower(1);
                    rightExtendPosition = liftExtendMotorRight.getCurrentPosition() - 750;
                    leftExtendPosition = liftExtendMotorLeft.getCurrentPosition() - 750;
                    sleep(1);
                    liftExtendMotorRight.setTargetPosition(rightExtendPosition);
                    liftExtendMotorLeft.setTargetPosition(leftExtendPosition);
                }
            }

            if (gamepad1.left_bumper) {
                if (rightExtendPosition < 10) {
                    //liftExtendMotorRight.setPower(0.85);
                    //liftExtendMotorLeft.setPower(0.85);
                    liftExtendMotorRight.setPower(1);
                    liftExtendMotorLeft.setPower(1);
                    rightExtendPosition = liftExtendMotorRight.getCurrentPosition() + 750;
                    leftExtendPosition = liftExtendMotorLeft.getCurrentPosition() + 750;
                    sleep(10);
                    liftExtendMotorRight.setTargetPosition(rightExtendPosition);
                    liftExtendMotorLeft.setTargetPosition(leftExtendPosition);
                }
            } else if (gamepad2.left_bumper) {
                if (rightExtendPosition < 10) {
                    //liftExtendMotorRight.setPower(0.85);
                    //liftExtendMotorLeft.setPower(0.85);
                    liftExtendMotorRight.setPower(1);
                    liftExtendMotorLeft.setPower(1);
                    rightExtendPosition = liftExtendMotorRight.getCurrentPosition() + 750;
                    leftExtendPosition = liftExtendMotorLeft.getCurrentPosition() + 750;
                    sleep(10);
                    liftExtendMotorRight.setTargetPosition(rightExtendPosition);
                    liftExtendMotorLeft.setTargetPosition(leftExtendPosition);
                }
            }

//front claw
            if (gamepad1.a) {
                if (frontClawClosed) {
                    LeftClawServo.setPosition(0.7);
                    RightClawServo.setPosition(0.3);
                    frontClawClosed = false;
                } else if (!frontClawClosed) {
                    LeftClawServo.setPosition(1);
                    RightClawServo.setPosition(0);
                    frontClawClosed = true;
                }
                sleep(200);

            } else if (gamepad2.a) {

                if (frontClawClosed) {
                    LeftClawServo.setPosition(0.5);
                    RightClawServo.setPosition(0.5);
                    frontClawClosed = false;
                } else if (!frontClawClosed) {
                    LeftClawServo.setPosition(1);
                    RightClawServo.setPosition(0);
                    frontClawClosed = true;
                }
                sleep(200);
            }


//Back Claw
            if (gamepad1.b) {
                if (backClawClosed) {
                    LeftBackServo.setPosition(0.3);
                    RightBackServo.setPosition(0.6);
                    backClawClosed = false;
                } else if (!backClawClosed) {
                    LeftBackServo.setPosition(0.8);
                    RightBackServo.setPosition(0.0);
                    backClawClosed = true;
                }
                sleep(200);

            }
            else if (gamepad2.b) {
                if (backClawClosed) {
                    LeftBackServo.setPosition(0.3);
                    RightBackServo.setPosition(0.6);
                    backClawClosed = false;
                } else if (!backClawClosed) {
                    LeftBackServo.setPosition(0.8);
                    RightBackServo.setPosition(0.0);
                    backClawClosed = true;
                }
                sleep(200);

            }






            if (gamepad1.x || gamepad2.x) {
                angle2 = angle;
                if (angle2 > 90)
                {
                    ClawRotateServo.setPosition(0.5 -  (90 - angle2) / 360);

                }
                else
                {
                    ClawRotateServo.setPosition(0.5 + ( (angle2 - 90) / 360));
                }
                telemetry.addData("Current Angle2", angle2);
                telemetry.update();
            }

            if (gamepad1.y || gamepad2.y) {
                ClawRotateServo.setPosition(0.5);
            }

            // Maunal beta cv
            if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if(ClawRotateServo.getPosition() < 0.9) {
                    ClawRotateServo.setPosition(ClawRotateServo.getPosition() - 0.1);
                    sleep(250);
                }
            }

            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                if (ClawRotateServo.getPosition() > 0.1) {
                    ClawRotateServo.setPosition(ClawRotateServo.getPosition() + 0.1);
                    sleep(250);
                }
            }


            telemetry.addData("Current Angle", angle);
            telemetry.addData("Lift Rotate Position", liftRotateMotorRight.getCurrentPosition());
            telemetry.addData("Lift Extend Position", liftExtendMotorRight.getCurrentPosition());
            telemetry.addData("Claw Rotate Position", ClawRotateServo.getPosition());
            telemetry.addData("Left Claw Position", LeftClawServo.getPosition());
            telemetry.addData("Right Claw Position", RightClawServo.getPosition());
            telemetry.addData("Motor Front Left Power", motorFrontLeft.getPower());
            telemetry.addData("Motor Front Right Power", motorFrontRight.getPower());
            telemetry.addData("Motor Back Left Power", motorBackLeft.getPower());
            telemetry.addData("Motor Back Right Power", motorBackRight.getPower());

            telemetry.addData("Override", override);
            telemetry.update();

        }


        controlHubCam.stopStreaming();

    }

    private void initOpenCV() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Pass both telemetry and this (LinearOpMode) to the pipeline constructor
        controlHubCam.setPipeline(new ContourDetectionPipeline(this));

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }


    // ContourDetectionPipeline class with angle passing
    static class ContourDetectionPipeline extends OpenCvPipeline {
        private final Telemetry telemetry;
        private final LinearOpMode opMode;


        public ContourDetectionPipeline(LinearOpMode opMode) {
            this.opMode = opMode;
            this.telemetry = opMode.telemetry;
        }


        @Override
        public Mat processFrame(Mat input) {
            Mat hsvMat = new Mat();
            Mat mask = new Mat();
            Mat hierarchy = new Mat();


            // Convert to HSV for better color filtering
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);


            // Define lower and upper bounds for red detection
            Scalar lowerBlue1 = new Scalar(100, 150, 70);
            Scalar upperBlue1 = new Scalar(120, 255, 255);
            Scalar lowerBlue2 = new Scalar(121, 150, 70);
            Scalar upperBlue2 = new Scalar(140, 255, 255);


            Mat mask1 = new Mat();
            Mat mask2 = new Mat();


            // Threshold blue color in two ranges
            Core.inRange(hsvMat, lowerBlue1, upperBlue1, mask1);
            Core.inRange(hsvMat, lowerBlue2, upperBlue2, mask2);


            // Combine masks
            Core.add(mask1, mask2, mask);


            // Find contours in the mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            double maxLength = 0;
            double angle = 0;
            Point bestP1 = null, bestP2 = null;

            for (MatOfPoint contour : contours) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f approxCurve = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, approxCurve, 0.02 * Imgproc.arcLength(contour2f, true), true);

                Point[] points = approxCurve.toArray();
                for (int i = 0; i < points.length; i++) {
                    Point p1 = points[i];
                    Point p2 = points[(i + 1) % points.length]; // Next point in contour

                    double dx = p2.x - p1.x;
                    double dy = p2.y - p1.y;
                    double length = Math.hypot(dx, dy);

                    // Ensure it's a diagonal edge (not purely horizontal or vertical)
                    if (Math.abs(dx) > 10 && Math.abs(dy) > 10 && length > maxLength) {
                        maxLength = length;
                        bestP1 = p1;
                        bestP2 = p2;
                        angle = Math.toDegrees(Math.atan2(dy, dx));

                        // Normalize the angle to be within [0, 180]
                        angle = (angle + 180) % 180;
                    }
                }
            }

            // Draw the longest diagonal contour on the image
            if (bestP1 != null && bestP2 != null) {
                Imgproc.line(input, bestP1, bestP2, new Scalar(0, 255, 0), 3);
            }

            // Update the OpMode's angle
            telemetry.addData("Detected Contours", contours.size());
            telemetry.addData("Stable Angle", angle);
            telemetry.addData("Longest Diagonal Length", maxLength);
            telemetry.update();

            if (opMode instanceof DriverBlue) {
                ((DriverBlue) opMode).angle = angle;
            }

            return input;
        }
    }


}

 */