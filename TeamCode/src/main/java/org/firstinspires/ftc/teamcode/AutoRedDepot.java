package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name = "Distance", group = "none")
public class AutoRedDepot extends LinearOpMode {

    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public VuforiaTrackable stoneTarget;
    public VuforiaTrackable redRearBridge;
    public VuforiaTrackable blueRearBridge;
    public VuforiaTrackable redFrontBridge;
    public VuforiaTrackable blueFrontBridge;
    public VuforiaTrackable red1;
    public VuforiaTrackable red2;
    public VuforiaTrackable front1;
    public VuforiaTrackable front2;
    public VuforiaTrackable blue1;
    public VuforiaTrackable blue2;
    public VuforiaTrackable rear1;
    public VuforiaTrackable rear2;

    public VuforiaLocalizer vuforia;

    public DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        com.vuforia.Vuforia VU = new com.vuforia.Vuforia();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters param = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        com.vuforia.Vuforia.setInitParameters(null, 3, "" );
        CameraDevice.getInstance().setField("iso","100");
        param.vuforiaLicenseKey = "AbYPrgD/////AAAAGbvKMH3NcEVFmPLgunQe4K0d1ZQi+afRLxricyooCq+sgY9Yh1j+bBrd0CdDCcoieA6trLCKBzymC515+Ps/FECtXv3+CTW6fg3/3+nvKZ6QA18h/cNZHg5HYHmghlcCgVUmSzOLRvdOpbS4S+0Y/sWGXwFK0PbuGPSN82w8XPDBoRYSWjAf8GXeitmNSlm9n4swrMoYNpMDuWCDjSm1kWnoErjFA9NuNoFzAgO+C/rYzoYjTJRk40ETVcAsahzatRlP7PJCvNNXiBhE6iVR+x7lFlTZ841xifOIOPkfVc54olC5XYe4A5ZmQ6WFD03W5HHdQrnmKPmkgcr1yqXAJ3rLTK8FZK3KVgbxz3Eeqps0";
        //param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        param.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        com.vuforia.Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 13);

        vuforia = ClassFactory.getInstance().createVuforia(param);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        targetsSkyStone.activate();

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("stoneTarget");
        redRearBridge = targetsSkyStone.get(1);
        redRearBridge.setName("redRearBridge");
        blueRearBridge = targetsSkyStone.get(2);
        blueRearBridge.setName("blueRearBridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("redFrontBridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("blueFrontBridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("red1");
        red2 = targetsSkyStone.get(6);
        red2.setName("red2");
        front1 = targetsSkyStone.get(7);
        front1.setName("front1");
        front2 = targetsSkyStone.get(8);
        front2.setName("front2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("blue1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("blue2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("rear1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("rear2");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");




        while (true)
        {
            moveToSkyStone();
            telemetry.update();
        }
    }

    public float aquireStoneTarget()
    {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) stoneTarget.getListener();


        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2);
            telemetry.addData("Distance:", "%f", d);
            Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
            OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
            Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

            return d;
        }
        return 0;
    }

    public void getObstacleDistance() {
        double d = distanceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Obstacle Distance: ", "%f", d);
    }


    public void moveToSkyStone()
    {
        float d = aquireStoneTarget();

        while (Math.abs(d) > 220) {
            fl.setPower(0.3);
            bl.setPower(-0.3);
            fr.setPower(-0.3);
            br.setPower(0.3);

            d = aquireStoneTarget();
        }

        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }
}
