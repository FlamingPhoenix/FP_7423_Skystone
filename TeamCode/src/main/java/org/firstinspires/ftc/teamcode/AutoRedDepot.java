package org.firstinspires.ftc.teamcode;

import android.util.Log;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.PositionToImage;

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

    public float PPR = 1120F;  // 560 for new robot 1120 for old robot
    protected LinearOpMode op;


    protected PositionToImage lastKnownPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        lastKnownPosition = new PositionToImage(); //instantiate this first

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

        waitForStart();
        StrafeToImage(0.3f, stoneTarget, this, 10);
    }


    private float Max(float x1, float x2, float x3, float x4) {
        x1 = Math.abs(x1);
        x2 = Math.abs(x2);
        x3 = Math.abs(x3);
        x4 = Math.abs(x4);
        float m = x1;

        if (x2 > m)
            m = x2;
        if (x3 > m)
            m = x3;
        if (x4 > m)
            m = x4;

        return m;
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

    public void StopAll(){
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void Strafe(float power, float distance, Direction d /*, OpMode op*/) {

        float x = (2.0f*PPR * distance)/(4F * (float)Math.PI); // used to be a 2 at top. tried 1.5, seems ok
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        int initialPosition = fl.getCurrentPosition();
        int positionDiff = 0;

        while (positionDiff < targetEncoderValue && op.opModeIsActive()) {
            /*
            op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            positionDiff = Math.abs(fl.getCurrentPosition() - initialPosition);
            //Log.i("[Phoenix]:encoder #", Integer.toString(positionDiff));

            //if(currentPosition < 200)
            //actualPower = .28F;

            float flPower, frPower, blPower, brPower;

            flPower = actualPower * 1.2F - 0.1f*power; //0.05F; when strafe to left, actual power is negative, but power remains positive.
            frPower = -actualPower * 1.2F - 0.1f*power; //0.05F;
            blPower = -actualPower * 1F - 0.1f*power; //0.05F;
            brPower = actualPower * 1F - 0.1f*power; //0.05F;

            float max = Max(flPower, frPower, blPower, brPower);

            fl.setPower(flPower/max);
            fr.setPower(frPower/max);
            bl.setPower(blPower/max);
            br.setPower(brPower/max);
        }

        StopAll();
    }

    public void Drive(float power, float distance, Direction d) {

        float x = (PPR * distance)/(4F * (float)Math.PI);
        int targetEncoderValue = Math.round(x);


        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        //added code below to support reverse driving, tested Oct 29, Erik did ofc this

        if (d == Direction.BACKWARD) {
            power = -1 * power;
        }

        while (currentPosition < targetEncoderValue && op.opModeIsActive()) {

            currentPosition = (Math.abs(fl.getCurrentPosition()));
            //Log.i("[Phoenix]:encoder #", Integer.toString(currentPosition));
            fl.setPower(power);
            fr.setPower(power);
            bl.setPower(power);
            br.setPower(power);
        }

        StopAll();

    }

    public void Turn(float power, int angle, Direction d, MyBoschIMU imu, LinearOpMode opMode) {

        Orientation startOrientation = imu.resetAndStart(d);

        float targetAngle;
        float currentAngle;
        float actualPower = power;
        float stoppingAngle = 0;

        if (d == Direction.CLOCKWISE) {
            actualPower = -(power);

            targetAngle = startOrientation.firstAngle - angle;
            currentAngle = startOrientation.firstAngle;

            while ((currentAngle - stoppingAngle) > targetAngle  && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v =  imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs(( 0.2648f * speed) - 7f);
                Log.i("[phoenix:turnTest]", String.format("StartingAngle=%f, CurrentAngle=%f, AngularVelocity=%f, StoppingAngle=%f", startOrientation.firstAngle, currentAngle, speed, stoppingAngle));

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }
        else {
            actualPower = power;

            targetAngle = startOrientation.firstAngle + angle;
            currentAngle = startOrientation.firstAngle;
            while ((currentAngle + stoppingAngle) < targetAngle  && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v =  imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs(( 0.2648f * speed) - 7f); //-8.5609

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }


        StopAll();

    }


    public void StrafeToImage(float power, VuforiaTrackable imageTarget, LinearOpMode opMode, float safetyDistance) {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();

        float actualPower = power;

        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            float x = pos.getColumn(3).get(1);
            float additionalpower = 0;


            while ((Math.abs(d) >= 200) && (imageListener.isVisible()) && opMode.opModeIsActive()) {
                pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();

                Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
                OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
                Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                opMode.telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

                //Keep track the last known location
                lastKnownPosition.translation = pos.getTranslation();
                lastKnownPosition.orientation = adjustedOrientation;

                d = lastKnownPosition.translation.get(2);
                x = lastKnownPosition.translation.get(1);

                opMode.telemetry.addData("x: ", "x = %f", x);

                float distanceAdjustment = Math.abs(d);
                if (distanceAdjustment > 1200F)
                    distanceAdjustment = 1200F;
                else if (distanceAdjustment < 700F)
                    distanceAdjustment = 0;

                if (x > 15) {
                    additionalpower = actualPower * 0.5F * (Math.abs(x) / 150F) * ((1200F - distanceAdjustment) / 1200F);
                } else if (x < -15) {
                    additionalpower = actualPower * -0.5F * (Math.abs(x) / 150F) * ((1200F - distanceAdjustment) / 1200F);
                }

                float flTurnAdjust = 0;
                float blTurnAdjust = 0;
                float rotationMargin = 3f * Math.abs(d) / 10f;


                if (adjustedOrientation.firstAngle < -3 && x > -rotationMargin) {
                    flTurnAdjust = actualPower * 1.25F * (Math.abs(adjustedOrientation.firstAngle) / 40F) * ((1200F - distanceAdjustment) / 1200F);
                } else if (adjustedOrientation.firstAngle > 3 && x < rotationMargin) {
                    blTurnAdjust = actualPower * -1.25F * (Math.abs(adjustedOrientation.firstAngle) / 40F) * ((1200F - distanceAdjustment) / 1200F);
                }

                float flPower, frPower, blPower, brPower;

                flPower = actualPower + additionalpower + flTurnAdjust;
                frPower = (-actualPower + additionalpower) * 1.2F;
                blPower = -actualPower + additionalpower + blTurnAdjust;
                brPower = actualPower + additionalpower;

                float max = Max(flPower, frPower, blPower, brPower);

                fl.setPower(flPower / max);
                fr.setPower(frPower / max);
                bl.setPower(blPower / max);
                br.setPower(brPower / max);

                Log.i("[phoenix:StrafeToImage]", String.format("x = %f, d = %f, addpower = %f, actpower = %f, distadj = %f", x, d, additionalpower, actualPower, distanceAdjustment));
                // Log.i("[phoenix:StrafeToImage]", String.format("raw x=%f, y=%f, z=%f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle));
                Log.i("[phoenix:StrafeToImage]", String.format("adj x=%f, y=%f, z=%f, flTurnAdjust=%f, blTurnAdjust=%f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle, flTurnAdjust, blTurnAdjust));
                opMode.telemetry.update();
            }
        }
        else {
            this.Strafe(.4F, safetyDistance, Direction.RIGHT);
            StopAll();
            return;
        }
        StopAll();
        float remainDistance = (Math.abs(lastKnownPosition.translation.get(2)) - 100) * .0254F;
        opMode.telemetry.addData("Remaining Distance: ", "x = %f", remainDistance);
        if (remainDistance > 4F)
            this.Strafe(0.5F, remainDistance, Direction.RIGHT);
        opMode.telemetry.update();
    }
}
