package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name="Red Near NOT Ideal", group="Red ILT")
public class ILTRedCloseNonIdeal extends NextFTCOpMode {

    // conditions + hardware

    boolean autoShoot = false;
    MotorEx turret = new MotorEx("turret").zeroed().reversed();
    private ControlSystem controller;

    MotorEx frontLeftMotor;
    MotorEx frontRightMotor;
    MotorEx backLeftMotor;
    MotorEx backRightMotor;

    Limelight3A limelight;
    IMU imu;
    Follower clanka;

    Rev2mDistanceSensor proxySens;
    RevColorSensorV3 colorSens;
    Servo backlight;
    boolean hasArtifact;

    boolean shooting = false;

    // turret vars
    boolean turretTracking = true;

    double fx = 626.909;
    double fy = 626.909;

    double cx = 426.007;
    double cy = 236.834;

    double robotYaw = 0;

    double globalCameraYaw = 0;

    Boolean blobDetected;

    //define Color Ranges
    ColorRange greenRange = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(15, 0, 0),     // Min (Y, Cr, Cb)
            new Scalar(255, 127, 170)   // Max (Y, Cr, Cb)
    );

    //custom PURPLE Range (YCrCb), these values are NOT final
    //purple is generally high in Red (Cr) and high in Blue (Cb)
    ColorRange purpleRange = new ColorRange(
            ColorSpace.YCrCb,
            new Scalar(32, 130, 140),  // Min (Y, Cr, Cb)
            new Scalar(255, 170, 200)   // Max (Y, Cr, Cb)
    );

    double clankerX, clankerY, clankerR, heading;
    boolean running;
    double targetVel;
    double targetHood = 0;
    double yaw;
    double distance;
    double delay;
    double blueX = 0;
    double blueY = 144;
    double offset = 8.8;
    double goalX = 7.1;
    double goalY = 144;
    double visionX = 0;
    double visionY = 0;
    double theta;
    double ticks;
    double ticksToDegrees = (130.0 / 36.0) * (384.5 / 360.0);
    double robotHeadingDegrees = 0;
    double leftMax = 691;
    double rightMax = -809;
    double xDiff = 0;
    double yDiff = 0;
    double limelightX = 0;
    double limelightY = 0;
    double angleOffset = 260;
    boolean limelightTracking = false;
    boolean hasCorrectedLL = false;
    double finalTargetTicks;
    boolean far = false;
    boolean readyShoot = false;

    double robotVelocityMag = 0;
    double robotVelocityXComp = 0;
    double robotVelocityYComp = 0;

    CRServoEx liftL;
    CRServoEx liftR;

    // visionmaxxing init
    ColorBlobLocatorProcessor purpleProcessor;
    ColorBlobLocatorProcessor greenProcessor;
    VisionPortal portal;

    public ILTRedCloseNonIdeal() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        multiFunctionSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        transferSubsystem.INSTANCE,
                        hoodSubsystem.INSTANCE,
                        turretSubsystem.INSTANCE // used for kinematic turret logic
                ),
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    // velo/hood calcs

//    public double getDistance(double ta) {
//        return 75.16142 * Math.pow(ta, -0.47932);
//    }
//    public double getHoodAngle(double dist) {
//        return 0.00578881 * dist - 0.00802588;
//    }
//
//    public double getVelocity(double dist) {
//        return 10.99866 * dist + 1084.95409;
//    }

    //large triangle curves
    public double getVel(double dist) {
        return 7.65268 * dist + 1123.06996;
    }

    public double getHood(double dist) {
        return 71.71201 * Math.pow(dist, -1.17652);
    }

    //small triangle curves
    public double getVelFar(double dist){
        return 19.17589 * dist - 582.51652;
    }
    public double getHoodFar(double dist){
        return -0.0210202 * dist + 3.48553;
    }



    // turret calcs
    public double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double atan2(double y, double x) {
        return Math.atan2(y, x);
    }

    public double calcDigger() {
        return atan2(goalY - (7 + clankerY), goalX - clankerX);
    }

    public double H2(double digger) {
        return normalize(digger - heading) * 180 / Math.PI;
    }

    public double tickAdjustment(double digger) {
        double calcTicks = H2(digger) * (130.0 / 36.0) * (384.5 / 360.0);
        double ticksPerRev = 384.5 * (130.0 / 36.0);

        if (leftMax != 0 && calcTicks > leftMax) calcTicks -= ticksPerRev;
        else if (rightMax != 0 && calcTicks < rightMax) calcTicks += ticksPerRev;

        return calcTicks;
    }

    @Override
    public void onInit() {
        liftL = new CRServoEx("liftLeft");
        liftR = new CRServoEx("liftRight");

        imu = hardwareMap.get(IMU.class, "imu");

        proxySens = hardwareMap.get(Rev2mDistanceSensor.class, "proxySensor");
        colorSens = hardwareMap.get(RevColorSensorV3.class, "color sensor");
        backlight = hardwareMap.get(Servo.class, "backlight");

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        imu.initialize(new IMU.Parameters(revOrientation));
        imu.resetYaw();

        hoodSubsystem.INSTANCE.goon(1);
        purpleProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(purpleRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false) //set to false for comp for faster processing
                .setBlurSize(6)
                .build();

        greenProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(greenRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(false)
                .setBlurSize(6)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(purpleProcessor)
                .addProcessor(greenProcessor)
                .setCameraResolution(new Size(864, 480)) // Your specified resolution
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .build();

        frontLeftMotor = new MotorEx("leftFront");
        frontRightMotor = new MotorEx("rightFront");
        backLeftMotor = new MotorEx("leftRear");
        backRightMotor = new MotorEx("rightRear");

        frontRightMotor.setDirection(1);
        backRightMotor.setDirection(1);
        frontLeftMotor.setDirection(1);
        backLeftMotor.setDirection(1);

        clanka = PedroComponent.follower();
        clanka.setStartingPose(new Pose(79.6702, 91.5592, .783).mirror());

        transferSubsystem.INSTANCE.toNeutral.schedule();

        turret.setCurrentPosition(0);
        turret.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        controller = ControlSystem.builder()
                .posPid(0.016, 0.0, 0.0001)
                .basicFF(0, 0, 0.1)
                .build();
        controller.setGoal(new KineticState(0.0));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        telemetry.setMsTransmissionInterval(1);

        targetVel = 0;
    }

    @Override
    public void onStartButtonPressed() {
        limelight.start();

        //drive
        new MecanumDriverControlled(
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        ).schedule();

        button(() -> gamepad1.y).whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

    }

    @Override
    public void onUpdate() {
        clanka.update();
        BindingManager.update();

        if ((proxySens.getDistance(DistanceUnit.INCH) < 5) && colorSens.getDistance(DistanceUnit.INCH) < 5) {
            hasArtifact = true;
            backlight.setPosition(4.720);
        }
        else {
            hasArtifact = false;
            backlight.setPosition(0.277);
        }

        blobDetected = checkForBlobs(portal, purpleProcessor, greenProcessor);
        // robot pose(X, Y, R)
        heading = clanka.getHeading();
        clankerX = clanka.getPose().getX();
        clankerY = clanka.getPose().getY();

        if(clankerY < 40){
            far = true;
        }
        else{
            far = false;
        }

        robotVelocityMag = clanka.getVelocity().getMagnitude();
        robotVelocityXComp = clanka.getVelocity().getXComponent();
        robotVelocityYComp = clanka.getVelocity().getMagnitude();

        if(Math.abs(robotVelocityMag) < 10){
            readyShoot = true;
        }
        else{
            readyShoot = false;
        }

        // ll digga
        LLResult result = limelight.getLatestResult();
        boolean tagFound = (result != null && result.isValid());


        distance = Math.sqrt(Math.pow((blueX - clankerX), 2) + Math.pow((blueY - clankerY), 2)) + offset;

        if(readyShoot) {
            if (!hasCorrectedLL) {
                if (tagFound) {
                    yaw = result.getTx();
                    double relativeTickOffset = yaw * ticksToDegrees;
                    finalTargetTicks = turret.getCurrentPosition() + relativeTickOffset;
                    hasCorrectedLL = true;
                } else {
                    theta = H2(calcDigger());
                    ticks = tickAdjustment(calcDigger());
                    finalTargetTicks = -ticks;
                }
            }
        }
        else {
            theta = H2(calcDigger());
            ticks = tickAdjustment(calcDigger());
            finalTargetTicks = -ticks;
            hasCorrectedLL = false;
        }

        // subsystem automation velo+hood
        if (autoShoot && (gamepad1.right_trigger > 0.1)) {
            if(!far){
                outtakeSubsystem.INSTANCE.setVel(getVel(distance)).schedule();
            }
            else{
                outtakeSubsystem.INSTANCE.setVel(getVelFar(distance)).schedule();
            }
        }
        else if(robotVelocityMag > 10){
            outtakeSubsystem.INSTANCE.off().schedule();
        }

        if (gamepad1.a) {
            turretTracking = !turretTracking;
        }

        if (turretTracking) {
            controller.setGoal(new KineticState(finalTargetTicks));
        } else {
            //on opposite toggle, set pos to 0
            controller.setGoal(new KineticState(0));
        }

        KineticState currentState = new KineticState(turret.getCurrentPosition(), turret.getVelocity());
        double turretPower = controller.calculate(currentState);

        if(!far){
            hoodSubsystem.INSTANCE.goon(getHood(distance)).schedule();
        }else{
            hoodSubsystem.INSTANCE.goon(getHoodFar(distance)).schedule();
        }

        if (!turretTracking && Math.abs(turret.getCurrentPosition()) < 5) {
            turret.setPower(0);
        } else {
            turret.setPower(turretPower);
        }

        //auto shoot algorithm
        //first, check if turret has corrected already(wraps turret error, robot velocity)
        if(hasCorrectedLL){
            autoShoot = true;
            if(hasArtifact){
                multiFunctionSubsystem.INSTANCE.transpherSequencNiga().schedule();
            }
        }

        // intake digger
        if (gamepad1.left_bumper) {
            intakeSubsystem.INSTANCE.slowSpit.schedule();
        } else if (gamepad1.right_bumper) {
            intakeSubsystem.INSTANCE.eat.schedule();
        }
        else if (blobDetected != null) {
            if (blobDetected) {
                intakeSubsystem.INSTANCE.eat.schedule();
            }
            else {
                intakeSubsystem.INSTANCE.sleep.schedule();
            }
        }
        else {
            intakeSubsystem.INSTANCE.sleep.schedule();
        }


        if (gamepad1.dpad_up) {
            liftL.setPower(1);
            liftR.setPower(-1);
        } else if (gamepad1.dpad_down) {
            liftL.setPower(-1);
            liftR.setPower(1);
        } else {
            liftL.setPower(0);
            liftR.setPower(0);
        }

        if (gamepad1.dpad_up) {
            limelightTracking = !limelightTracking;
        }

        telemetry.addData("Distance", distance);
        telemetry.addData("Tag Found", tagFound);
        telemetry.addData("Turret Pos", turret.getCurrentPosition());
        telemetry.addData("Degree Offset (Limelight)", yaw);
        telemetry.addData("Degree Offset (Kinematics)", theta);
        telemetry.addData("Target Ticks", -ticks);
        telemetry.addData("robotVelocity", clanka.getVelocity());


        // List<ColorBlobLocatorProcessor.Blob> purpleBlobs = purpleProcessor.getBlobs();
        // List<ColorBlobLocatorProcessor.Blob> greenBlobs = greenProcessor.getBlobs();

        // Apply filter just for display count (logic is handled in function below)
        // ColorBlobLocatorProcessor.Util.filterByArea(30, 20000, purpleBlobs);
        // ColorBlobLocatorProcessor.Util.filterByArea(30, 20000, greenBlobs);

        // telemetry.addData("Purple Count", purpleBlobs.size());
        // telemetry.addData("Green Count", greenBlobs.size());
        // telemetry.addData("Balls Found:", blobDetected);
        telemetry.update();
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
        boolean seePurple = purpleBlobs.size() > 1;
        boolean seeGreen = greenBlobs.size() > 3;

        // 5. Return logic
        return seePurple ||seeGreen;
    }
//        public YawPitchRollAngles getIMU(AngleUnit angleUnit){
//            return imu.getRobotYawPitchRollAngles();
//        }
}