package org.firstinspires.ftc.teamcode.opmode.Tests;

import static java.lang.Math.pow;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="Digger")
public class distanceLimelight extends LinearOpMode{

    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        telemetry.setMsTransmissionInterval(15);

        waitForStart();

        while (opModeIsActive()) {

            LLResult llresult = limelight.getLatestResult();
            LLStatus status = limelight.getStatus();

            double captureLatency = llresult.getCaptureLatency();
            double targetingLatency = llresult.getTargetingLatency();
            double parseLatency = llresult.getParseLatency();

            boolean tagFound = (llresult != null && llresult.isValid());
            if (tagFound) {
                telemetry.update();
            }

            telemetry.addData("Target Area", llresult.getTa());
            telemetry.addData("Target X", llresult.getTx());
            telemetry.addData("Distance (inches)", getDistance(llresult.getTa()));
            telemetry.addData("Botpose", llresult.getBotpose_MT2());
        }
    }

    public double getDistance(double ta) {
        return 75.16142 * Math.pow(ta, -0.47932);
    }
}
