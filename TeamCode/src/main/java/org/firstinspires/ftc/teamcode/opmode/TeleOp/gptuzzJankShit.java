package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import static dev.nextftc.bindings.Bindings.button;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.hoodSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.intakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.multiFunctionSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.outtakeSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Subsystems.transferSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled

@TeleOp(name = "gptuzzzzzzzz")
public class gptuzzJankShit extends NextFTCOpMode {


    public gptuzzJankShit() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        hoodSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE),


                BulkReadComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("leftFront");
    private final MotorEx frontRightMotor = new MotorEx("rightFront");
    private final MotorEx backLeftMotor = new MotorEx("leftRear");
    private final MotorEx backRightMotor = new MotorEx("rightRear");


    // public double fx = 1156.544;
    // public double fy = 867.408;
    // public double cx = 804.290;
    // public double cy = 332.259;

    //for 1920 x 1080
    public double fx = 1385.920;
    public double fy = 1385.920;
    public double cx = 951.982;
    public double cy = 534.084;
    public boolean calibratedCamera;

    private Follower follower;

    double yaw;
    public AprilTagProcessor aprilTagProcessor;

    private Paths paths;

    private HeadingPID headingPID;
    private boolean autoAimEnabled = false;
    private double targetHeading = 0; // radians, will set per depot


    //private final Pose autoAim = new Pose(follower.get, Math.toRadians(180));


    @Override
    public void onInit() {

        // 1️⃣ initialize the follower first
        follower = Constants.createFollower(hardwareMap);

        // 2️⃣ create paths using the follower
        paths = new Paths(follower);

        headingPID = new HeadingPID(0.9, 0.0, 0.01); // tune kP/kI/kD


        // 3️⃣ set the follower starting pose
        follower.setStartingPose(paths.startPose);

        multiFunctionSubsystem.INSTANCE.stopPlease().schedule();
        transferSubsystem.INSTANCE.toNeutral.schedule();


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
        waitForStart();


    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()

        );

        driverControlled.schedule();

//        Gamepads.gamepad1().a()
//                .whenBecomesTrue(
//                        multiFunctionSubsystem.INSTANCE.outtakeSequence()
//                );

        Gamepads.gamepad1().leftTrigger().atLeast(0.1).whenTrue(
                outtakeSubsystem.INSTANCE.off()
        );

        Gamepads.gamepad1().rightTrigger().atLeast(0.1)
                .whenBecomesTrue(outtakeSubsystem.INSTANCE.mid())
                .whenTrue(outtakeSubsystem.INSTANCE.mid())
                .whenBecomesFalse(outtakeSubsystem.INSTANCE.off());

        Button gamepad1leftBumper = button(() -> gamepad1.left_bumper);
        gamepad1leftBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.spit)
                .whenTrue(intakeSubsystem.INSTANCE.spit)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);

        Button gamepad1rightBumper = button(() -> gamepad1.right_bumper);
        gamepad1rightBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.eat)
                .whenTrue(intakeSubsystem.INSTANCE.eat)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);


        Button gamepad1y = button(() -> gamepad1.y);
        gamepad1y
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

        Button gamepad1Dleft = button(() -> gamepad1.dpad_left);
        gamepad1Dleft
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.outtakeSequence());

        Button gamepad1a = button(() -> gamepad1.a);
        gamepad1a
                .whenBecomesTrue(hoodSubsystem.INSTANCE.one);

        Button gamepad1b = button(() -> gamepad1.b);
        gamepad1b
                .whenBecomesTrue(() -> {
                    autoAimEnabled = true;
                    // Choose depot heading: example 0 or 144 degrees
                    targetHeading = Math.toRadians(315);
                })
                .whenBecomesFalse(() -> autoAimEnabled = false);

        //.whenFalse(() -> follower.breakFollowing());



//        Button gamepad1Dup = button(() -> gamepad1.dpad_up);
//        gamepad1Dup
//                .whenBecomesTrue(
//                      new TurnToCommand(follower, yaw)
//                      )






        /*
        Gamepads.gamepad1().leftBumper().whenTrue(
                intakeSubsystem.INSTANCE.eat
        );

        Gamepads.gamepad1().leftBumper().whenTrue(
                intakeSubsystem.INSTANCE.spit
        );

         */

//        Gamepads.gamepad1().x().whenBecomesTrue(
//                transferSubsystem.INSTANCE.toOuttake
//        );
//        Gamepads.gamepad1().y().whenBecomesTrue(
//                transferSubsystem.INSTANCE.toNeutral
//        );




    }


    @Override
    public void onUpdate() {
        BindingManager.update();
        telemetry.addData("Current Velocity: ", outtakeSubsystem.INSTANCE.getJawn());
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            int id = detection.id;
            if (id == 20 || id == 24) {
                yaw = Math.toDegrees(detection.ftcPose.yaw);
                telemetry.addLine(String.valueOf(detection.ftcPose.yaw));
                telemetry.addLine(String.valueOf(detection.ftcPose.range));

                telemetry.update();
            }

        }

        // PID rotation
        double rotate = 0;
        if (autoAimEnabled) {
            double dt = 0.02; // assuming 50Hz update
            double imuHeading = follower.getHeading(); // radians
            rotate = headingPID.update(targetHeading, imuHeading, dt);
        }

        // drive mecanum
        frontLeftMotor.setPower(rotate);
        frontRightMotor.setPower(-1 * rotate);
        backLeftMotor.setPower(rotate);
        backRightMotor.setPower(-1 * rotate);

        // telemetry
        telemetry.addData("Heading", Math.toDegrees(follower.getHeading()));
        telemetry.update();

        telemetry.addData("Yaw PP IMU: ", follower.getHeading());
        telemetry.update();
        follower.update();

        super.onUpdate();// updates command bindings
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        super.onStop();
    }

    public class HeadingPID {
        private double kP, kI, kD;
        private double integral = 0;
        private double lastError = 0;

        public HeadingPID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double update(double target, double current, double dt) {
            double error = normalizeAngle(target - current);
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;
            return kP * error + kI * integral + kD * derivative;
        }

        // Normalize to [-pi, pi] range
        private double normalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2*Math.PI;
            while (angle < -Math.PI) angle += 2*Math.PI;
            return angle;
        }
    }



    public class Paths {

        public PathChain autoAim;
        public Pose startPose = new Pose(48, 48, Math.toRadians(90));

        public Paths(Follower follower) {
            autoAim = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    (follower::getPose), (follower::getPose))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }
}