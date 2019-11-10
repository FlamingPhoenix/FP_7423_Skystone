package org.firstinspires.ftc.teamcode;

import android.service.dreams.DreamService;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Direction;
import org.firstinspires.ftc.teamcode.Library.ImageNavigation;
import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;
import org.firstinspires.ftc.teamcode.MyClass.PositionToImage;
import org.firstinspires.ftc.teamcode.MyClass.SkystonePosition;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public abstract class AutoBase extends LinearOpMode {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;

    public Servo twister;
    public DistanceSensor stoneScanner;

    //public DistanceSensor distanceSensor;

    public float PPR = 1120F; //changed ppr for test robot
    public float diameter = 3F;
    public MyBoschIMU imu;

    DcMotor intakeMotorLeft;
    DcMotor intakeMotorRight;


    protected PositionToImage lastKnownPosition;

    ImageNavigation imageNavigation;

    public int primaryAngle;

    public void initialize() {
        lastKnownPosition = new PositionToImage(); //instantiate this first

        fl = hardwareMap.dcMotor.get("frontleft");
        fr = hardwareMap.dcMotor.get("frontright");
        bl = hardwareMap.dcMotor.get("backleft");
        br = hardwareMap.dcMotor.get("backright");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        imageNavigation = new ImageNavigation(hardwareMap, this);

        imu = new MyBoschIMU(hardwareMap);
        imu.initialize(new BNO055IMU.Parameters());

       // distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        intakeMotorLeft = hardwareMap.dcMotor.get("intaketh1");
        intakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorRight = hardwareMap.dcMotor.get("intaketh2");
        intakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        primaryAngle = (int)imu.getAngularOrientation().firstAngle;
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

    public float aquireStoneTarget() {
        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageNavigation.stoneTarget.getListener();


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

    //used to test the rotating stone distance sensor

//    public double getObstacleDistance() {
//        double d = distanceSensor.getDistance(DistanceUnit.INCH);
//        telemetry.addData("Obstacle Distance: ", "%f", d);
//        return d;
//    }
//
//    public double DriveUntilDistance(float safetyDistance, float robotStartingAngle, MyBoschIMU imu) {
//        float robotCurrentAngle;
//        double d = distanceSensor.getDistance(DistanceUnit.INCH);
//
//        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
//        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        while (d > safetyDistance) {
//            robotCurrentAngle = imu.getAngularOrientation().firstAngle;
//            if (robotCurrentAngle - robotStartingAngle >= 93) { // 3 degrees or more
//                fl.setPower(-0.3);
//                fr.setPower(0.3);
//                bl.setPower(0.5);
//                br.setPower(-0.5);
//            } else if (robotCurrentAngle - robotStartingAngle <= 87) { // -3 degrees or more
//                fl.setPower(-0.5);
//                fr.setPower(0.5);
//                bl.setPower(0.3);
//                br.setPower(-0.3);
//            } else {
//                fl.setPower(-0.3);
//                fr.setPower(0.3);
//                bl.setPower(0.3);
//                br.setPower(-0.3);
//            }
//
//            d = distanceSensor.getDistance(DistanceUnit.INCH);
//
//            telemetry.addData("Obstacle Distance: ", "%f", d);
//            telemetry.update();
//        }
//        StopAll();
//        double driveDistance = fl.getCurrentPosition();
//        return driveDistance;
//    }

    public void StopAll() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void Strafe(float power, float distance, Direction d /*, OpMode op*/) {

        imu.resetAndStart(Direction.COUNTERCLOCKWISE);

        int robotStartingAngle = (int)imu.getAngularOrientation().firstAngle;

        float x = (2.0f * PPR * distance) / (diameter * (float) Math.PI); // used to be a 2 at top. tried 1.5, seems ok
        int targetEncoderValue = Math.round(x);

        float actualPower = power;
        if (d == Direction.LEFT)
            actualPower = -(power);

        int initialPosition = br.getCurrentPosition();
        int positionDiff = 0;

        while (positionDiff < targetEncoderValue && this.opModeIsActive()) {

            /* op.telemetry.addData("current:", currentPosition);
            op.telemetry.addData("target:", targetEncoderValue);
            op.telemetry.update();
            */
            int currentPosition = br.getCurrentPosition();
            Log.i("[phoenix]", String.format("Current:%10d Initial:%10d", currentPosition, initialPosition));
            positionDiff = Math.abs(currentPosition - initialPosition); //changed encoder for test bot
            //Log.i("[Phoenix]:encoder #", Integer.toString(positionDiff));

            //if(currentPosition < 200)
            //actualPower = .28F;

            float flPower, frPower, blPower, brPower;

            float robotCurrentAngle = imu.getAngularOrientation().firstAngle;

            float angleModify = power;

            if (d == Direction.LEFT)
            {
                if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower - angleModify;
                    brPower = actualPower + angleModify;
                }
                else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
                {
                    flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower - angleModify;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
                else
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
            }
            else if(d == Direction.RIGHT)
            {
                if (robotCurrentAngle - robotStartingAngle >= 3) // 3 degrees or more
                {
                    flPower = actualPower + angleModify; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower - angleModify;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
                else if (robotCurrentAngle - robotStartingAngle <= -3) // -3 degrees or more
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower - angleModify;
                    brPower = actualPower + angleModify;
                }
                else
                {
                    flPower = actualPower; //when strafe to left, actual power is negative, but power remains positive.
                    frPower = -actualPower;
                    blPower = -actualPower;
                    brPower = actualPower;
                }
            }
            else
            {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }

            float max = Max(flPower, frPower, blPower, brPower);

            fl.setPower(flPower / max);
            fr.setPower(frPower / max);
            bl.setPower(blPower / max);
            br.setPower(brPower / max);

        }

        StopAll();
    }

    public void Drive(float power, float distance, Direction d) {

        float x = (PPR * distance) / (diameter * (float) Math.PI);
        int targetEncoderValue = Math.round(x);


        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int currentPosition = 0;

        //added code below to support reverse driving, tested Oct 29, Erik did this

        if (d == Direction.BACKWARD) {
            power = -1 * power;
        }

        while (currentPosition < targetEncoderValue && opModeIsActive()) {

            currentPosition = (Math.abs(br.getCurrentPosition()));
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

            while ((currentAngle - stoppingAngle) > targetAngle && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v = imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs((0.3f * speed) - 7f);
                Log.i("[phoenix:turnTest]", String.format("StartingAngle=%f, CurrentAngle=%f, AngularVelocity=%f, StoppingAngle=%f", startOrientation.firstAngle, currentAngle, speed, stoppingAngle));

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        } else {
            actualPower = power;

            targetAngle = startOrientation.firstAngle + angle;
            currentAngle = startOrientation.firstAngle;
            while ((currentAngle + stoppingAngle) < targetAngle && opMode.opModeIsActive()) {

                opMode.telemetry.addData("start:", startOrientation.firstAngle);
                opMode.telemetry.addData("current:", currentAngle);
                opMode.telemetry.addData("target:", targetAngle);
                opMode.telemetry.update();

                currentAngle = imu.getAngularOrientation().firstAngle;
                AngularVelocity v = imu.getAngularVelocity();
                float speed = Math.abs(v.xRotationRate);
                stoppingAngle = Math.abs((0.2648f * speed) - 7f); //-8.5609

                fl.setPower(-(actualPower));
                fr.setPower(actualPower);
                bl.setPower(-(actualPower));
                br.setPower(actualPower);
            }
        }
        StopAll();
    }



    public boolean StrafeToImage(float power, VuforiaTrackable imageTarget, LinearOpMode opMode, float safetyDistance, double stopDistance, int targetAngle) {

        stopDistance = stopDistance * 25.4;

        VuforiaTrackableDefaultListener imageListener = (VuforiaTrackableDefaultListener) imageTarget.getListener();

        float actualPower = power;

        if (imageListener.isVisible()) {
            OpenGLMatrix pos = imageListener.getPose();
            float d = pos.getColumn(3).get(2); //distance to the image in millimeter;
            float x = pos.getColumn(3).get(1);
            float additionalpower = 0;

            lastKnownPosition.translation = pos.getTranslation();

            Log.i("[phoenix]", String.format("distanceImage (before loop) = %10.2f", d));

            while ((Math.abs(d) >= stopDistance) && (imageListener.isVisible()) && opMode.opModeIsActive()) {

                pos = ((VuforiaTrackableDefaultListener) imageTarget.getListener()).getPose();
                d = pos.getColumn(3).get(2); //distance to the image in millimeter;
                x = -1 * pos.getColumn(3).get(1);

                Log.i("[phoenix]", String.format("distanceImage = %10.2f", d));

                Orientation orientation = Orientation.getOrientation(pos, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                OpenGLMatrix rotationMatrix = OpenGLMatrix.rotation(AxesReference.EXTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES, orientation.thirdAngle * -1, orientation.firstAngle * -1, 0);
                OpenGLMatrix adjustedPose = pos.multiplied(rotationMatrix);
                Orientation adjustedOrientation = Orientation.getOrientation(adjustedPose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                opMode.telemetry.addData("Angle: ", "x = %f, y = %f, z = %f", adjustedOrientation.firstAngle, adjustedOrientation.secondAngle, adjustedOrientation.thirdAngle);

                //Keep track the last known location
                lastKnownPosition.translation = pos.getTranslation();
                lastKnownPosition.orientation = adjustedOrientation;

                d = lastKnownPosition.translation.get(2);
                x = lastKnownPosition.translation.get(1) * -1; // bc camera rotation change

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
                float currentAngle = imu.getAngularOrientation().firstAngle;

                float angleDiff = currentAngle - primaryAngle;


                if (angleDiff < -3) {
                    flTurnAdjust = actualPower * -1.25F * (Math.abs(adjustedOrientation.firstAngle) / 40F) * ((1200F - distanceAdjustment) / 1200F);
                } else if (angleDiff > 3) {
                    blTurnAdjust = actualPower * 1.25F * (Math.abs(adjustedOrientation.firstAngle) / 40F) * ((1200F - distanceAdjustment) / 1200F);
                }

                float flPower, frPower, blPower, brPower;

                flPower = actualPower + additionalpower + flTurnAdjust;
                frPower = -actualPower + additionalpower;
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
            return false;
        }
        StopAll();
        opMode.telemetry.update();
        return true;
    }

    public void DriveToCoordinate(float x, float y) {
        double frPower = 0;
        double flPower = 0;
        double brPower = 0;
        double blPower = 0;

        float yDiff = -1000;
        float xDiff = 1000;

        while (yDiff < - 100 || Math.abs(xDiff) > 100)
        {
            frPower = 0;
            flPower = 0;
            brPower = 0;
            blPower = 0;

            OpenGLMatrix robotLocation = imageNavigation.getRobotLocation();

            if (robotLocation != null)
            {
                yDiff = y - robotLocation.getColumn(3).get(1);
                xDiff = x - robotLocation.getColumn(3).get(0);

                //Log.i("xydiff & xylocation", String.format("yDiff=%10.2f xDiff=%10.2f currentY=%10.2f currentX=%10.2f", yDiff/25.4, xDiff/25.4, robotLocation.getColumn(3).get(1)/25.4, robotLocation.getColumn(3).get(0)/25.4));
                Log.i("[phoenix]",  String.format("translateX = %10.2f, translateY = %10.2f, translateZ = %10.2f", robotLocation.getTranslation().get(0)/25.4, robotLocation.getTranslation().get(1)/25.4, robotLocation.getTranslation().get(2)/25.4));
                if (yDiff < -100)
                {
                    frPower = 0.3;
                    flPower = 0.3;
                    brPower = 0.3;
                    blPower = 0.3;
                }

                if (Math.abs(xDiff) > 100)
                {
                    double strafeAdjustmentPower = 0;

                    if (xDiff > 0)
                        strafeAdjustmentPower = -0.3;
                    else
                        strafeAdjustmentPower = 0.3;

                    frPower = frPower + strafeAdjustmentPower;
                    flPower = flPower - strafeAdjustmentPower;
                    brPower = brPower - strafeAdjustmentPower;
                    blPower = blPower + strafeAdjustmentPower;
                }

                fl.setPower(flPower);
                fr.setPower(frPower);
                bl.setPower(blPower);
                br.setPower(brPower);
            }

            else {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }

        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}

