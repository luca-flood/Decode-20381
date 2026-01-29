package org.firstinspires.ftc.teamcode.opmode.TeleOp.oldCode;

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
import org.firstinspires.ftc.teamcode.opmode.Subsystems.turretSubsystemLimelight;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled

@TeleOp(name = "NextFTC TeleOp Program Java Temp")
public class NEXTftcOpModeTemp extends NextFTCOpMode {


    public NEXTftcOpModeTemp() {
        addComponents(
                BindingsComponent.INSTANCE,
                new SubsystemComponent(
                        hoodSubsystem.INSTANCE,
                        intakeSubsystem.INSTANCE,
                        outtakeSubsystem.INSTANCE,
                        multiFunctionSubsystem.INSTANCE,
                        turretSubsystemLimelight.INSTANCE
                ),


                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
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

    double distance;
    public AprilTagProcessor aprilTagProcessor;

    private Paths paths;

    //private final Pose autoAim = new Pose(follower.get, Math.toRadians(180));


    @Override
    public void onInit() {

        follower = Constants.createFollower(hardwareMap);

        paths = new Paths(follower);

        PedroComponent.follower().setStartingPose(paths.startPose);

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

        frontLeftMotor.setDirection(-1);
        backLeftMotor.setDirection(-1);
        frontRightMotor.setDirection(-1);
        backRightMotor.setDirection(-1);


    }

    @Override
    public void onWaitForStart() {

    }

    @Override
    public void onStartButtonPressed() {
//        DriverControlledCommand driverControlled = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad1().leftStickY().negate(),
//                Gamepads.gamepad1().leftStickX(),
//                Gamepads.gamepad1().rightStickX()
//
//        );
//
//        DriverControlledCommand driverControlled2 = new MecanumDriverControlled(
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor,
//                Gamepads.gamepad2().leftStickY().negate(),
//                Gamepads.gamepad2().leftStickX(),
//                Gamepads.gamepad2().rightStickX()
//
//        );
//
//        driverControlled2.setScalar(0.2);
//
//        driverControlled.schedule();

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
                .whenBecomesTrue(() -> follower.activateHeading())
                .whenTrue(() -> follower.followPath(paths.autoAim))
                .whenBecomesFalse(() -> follower.breakFollowing());
        //.whenFalse(() -> follower.breakFollowing());

        Gamepads.gamepad1().leftTrigger().atLeast(0.1).whenTrue(
                outtakeSubsystem.INSTANCE.off()
        );

        Gamepads.gamepad1().rightTrigger().atLeast(0.1)
                .whenBecomesTrue(outtakeSubsystem.INSTANCE.mid())
                .whenTrue(outtakeSubsystem.INSTANCE.mid())
                .whenBecomesFalse(outtakeSubsystem.INSTANCE.off());

        Button gamepad2leftBumper = button(() -> gamepad2.left_bumper);
        gamepad2leftBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.spit)
                .whenTrue(intakeSubsystem.INSTANCE.spit)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);

        Button gamepad2rightBumper = button(() -> gamepad2.right_bumper);
        gamepad2rightBumper
                .whenBecomesTrue(intakeSubsystem.INSTANCE.eat)
                .whenTrue(intakeSubsystem.INSTANCE.eat)
                .whenBecomesFalse(intakeSubsystem.INSTANCE.sleep);


        Button gamepad2y = button(() -> gamepad2.y);
        gamepad2y
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.transpherSequencNiga());

        Button gamepad2Dleft = button(() -> gamepad2.dpad_left);
        gamepad2Dleft
                .whenBecomesTrue(multiFunctionSubsystem.INSTANCE.outtakeSequence());

        Button gamepad2a = button(() -> gamepad2.a);
        gamepad2a
                .whenBecomesTrue(hoodSubsystem.INSTANCE.one);

        Button gamepad2b = button(() -> gamepad2.b);
        gamepad1b
                .whenBecomesTrue(() -> follower.activateHeading())
                .whenTrue(() -> follower.followPath(paths.autoAim))
                .whenBecomesFalse(() -> follower.breakFollowing());

        /*
        Gamepads.gamepad1().leftBumper().whenTrue(
                intakeSubsystem.INSTANCE.eat
        );

        Gamepads.gamepad1().leftBumper().whenTrue(
                intakeSubsystem.INSTANCE.spit
        );

         */

//        Gamepads.gamepad1().x().whenBecomesTrue(
//                turretSubsystemLimelight.INSTANCE.getDaddy().setPower(-1);
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
                distance = detection.ftcPose.range;
                telemetry.addLine(String.valueOf(detection.ftcPose.range));



                telemetry.update();
            }

        }
        telemetry.addData("Yaw PP IMU: ", follower.getHeading());
        telemetry.addData("X", PedroComponent.follower().getPose().getX());
        telemetry.addData("Y", PedroComponent.follower().getPose().getY());
        telemetry.addData("Heading", PedroComponent.follower().getPose().getHeading());
        telemetry.update();
        follower.update();

        double y = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x * 1.1;
        double x = -gamepad1.left_stick_x;

        double y2 = gamepad2.left_stick_y;
        double rx2 = -gamepad2.right_stick_x * 1.1;
        double x2 = -gamepad2.left_stick_x;

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
            frontLeftMotor.setPower(frontLeftPower*0.95);
            backLeftMotor.setPower(backLeftPower*0.95);
            frontRightMotor.setPower(frontRightPower*0.95);
            backRightMotor.setPower(backRightPower*0.95);
        } else if (gamepad2.left_stick_y >= 0.1 || gamepad2.left_stick_x >= 0.1 || gamepad2.left_stick_y <= -0.1 || gamepad2.left_stick_x <= -0.1 || gamepad2.right_stick_y >= 0.1 || gamepad2.right_stick_x >= 0.1 || gamepad2.right_stick_y <= -0.1 || gamepad2.right_stick_x <= -0.1) {
            frontLeftMotor.setPower(frontLeftPower2 * 0.2);
            backLeftMotor.setPower(backLeftPower2 * 0.2);
            frontRightMotor.setPower(frontRightPower2 * 0.2);
            backRightMotor.setPower(backRightPower2 * 0.2);
        } else {
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
        }

        super.onUpdate();// updates command bindings
    }

    @Override
    public void onStop() {
        BindingManager.reset();
        super.onStop();
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
