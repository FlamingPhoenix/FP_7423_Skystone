package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp", group = "none")
public class MyTele extends OpMode {
    DcMotor fr;
    DcMotor fl;
    DcMotor br;
    DcMotor bl;
    float x1, x2, y1, y2;
    DcMotor intakeMotorLeft;
    DcMotor intakeMotorRight;
    DcMotor slideMotorLeft;
    DcMotor slideMotorRight;
    Servo pullerLeft;
    Servo pullerRight;

    Servo wrist;
    Servo finger;

    DistanceSensor backLeftDistanceSensor;
    DistanceSensor backRightDistanceSensor;
    long lastSampleTime;
    long currentSampleTime;

    double approachingSpeed;

    double lastDistanceLeft;
    double lastDistanceRight;

    public void drive(float x1, float y1, float x2) {
        if (backLeftDistanceSensor.getDistance(DistanceUnit.INCH) < 4 && backRightDistanceSensor.getDistance(DistanceUnit.INCH) < 4) {
            x1 /= 5;
            y1 /= 5;
            x2 /= 5;
        }

//        if (approachingSpeed > X) {
//
//        }

        float frontLeft = y1 + x1 + x2;
        float frontRight = y1 - x1 - x2;
        float backLeft = y1 - x1 + x2;
        float backRight = y1 + x1 - x2;

        frontLeft = Range.clip(frontLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        backLeft = Range.clip(backLeft, -1, 1);
        backRight = Range.clip(backRight, -1, 1);

        fl.setPower(frontLeft);
        fr.setPower(frontRight);
        bl.setPower(backLeft);
        br.setPower(backRight);
    }

    @Override
    public void init() {
        fr = hardwareMap.dcMotor.get("frontright");
        fl = hardwareMap.dcMotor.get("frontleft");
        br = hardwareMap.dcMotor.get("backright");
        bl = hardwareMap.dcMotor.get("backleft");

        intakeMotorLeft = hardwareMap.dcMotor.get("intaketh1");
        intakeMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorRight = hardwareMap.dcMotor.get("intaketh2");
        intakeMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotorLeft = hardwareMap.dcMotor.get("slideMotorLeft");
        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorRight = hardwareMap.dcMotor.get("slideMotorRight");
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        pullerLeft = hardwareMap.servo.get("pullerLeft");
        ServoControllerEx pullerLeftController = (ServoControllerEx) pullerLeft.getController();
        int pullerLeftServoPort = pullerLeft.getPortNumber();
        PwmControl.PwmRange pullerLeftPwmRange = new PwmControl.PwmRange(899, 1335);
        pullerLeftController.setServoPwmRange(pullerLeftServoPort, pullerLeftPwmRange);
        pullerLeft.setPosition(0);

        pullerRight = hardwareMap.servo.get("pullerRight");
        ServoControllerEx pullerRightController = (ServoControllerEx) pullerRight.getController();
        int pullerRightServoPort = pullerRight.getPortNumber();
        PwmControl.PwmRange pullerRightPwmRange = new PwmControl.PwmRange(899, 2105);
        pullerRightController.setServoPwmRange(pullerRightServoPort, pullerRightPwmRange);
        pullerRight.setPosition(0);

        wrist = hardwareMap.servo.get("wrist");
        ServoControllerEx wristController = (ServoControllerEx) wrist.getController();
        int wristServoPort = wrist.getPortNumber();
        PwmControl.PwmRange wristPwmRange = new PwmControl.PwmRange(899, 1475);
        wristController.setServoPwmRange(wristServoPort, wristPwmRange);
        wrist.setPosition(0.5);

        finger = hardwareMap.servo.get("finger");
        ServoControllerEx fingerController = (ServoControllerEx) finger.getController();
        int fingerServoPort = finger.getPortNumber();
        PwmControl.PwmRange fingerPwmRange = new PwmControl.PwmRange(899, 1700);
        fingerController.setServoPwmRange(fingerServoPort, fingerPwmRange);
        finger.setPosition(0);

        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftDistanceSensor = hardwareMap.get(DistanceSensor.class, "backLeftDistanceSensor");
        backRightDistanceSensor = hardwareMap.get(DistanceSensor.class, "backRightDistanceSensor");

        lastSampleTime = System.currentTimeMillis();
        lastDistanceLeft = backLeftDistanceSensor.getDistance(DistanceUnit.INCH);
    }

    @Override
    public void loop() {
        currentSampleTime = System.currentTimeMillis();
        if (currentSampleTime - lastSampleTime > 100) {
            double approachingSpeedLeft = (backLeftDistanceSensor.getDistance(DistanceUnit.INCH) - lastDistanceLeft) / (currentSampleTime - lastSampleTime);
            double approachingSpeedRight = (backLeftDistanceSensor.getDistance(DistanceUnit.INCH) - lastDistanceRight) / (currentSampleTime - lastSampleTime);
//            telemetry.addData("approachingSpeedLeft: ", "%f", approachingSpeedLeft);
//            telemetry.addData("approachingSpeedRight: ", "%f", approachingSpeedRight);
            telemetry.update();

           if (Math.abs(approachingSpeedLeft - approachingSpeedRight) < 2) {
                approachingSpeed = approachingSpeedLeft;
           }
            lastSampleTime = currentSampleTime;
        }

        x1 = gamepad1.left_stick_x;
        y1 = gamepad1.left_stick_y;
        x2 = gamepad1.right_stick_x;
        y2 = gamepad1.right_stick_y;

        double joystickLeftDistance = Math.pow(x1, 2) + Math.pow(y1, 2);
        if (joystickLeftDistance < 0.9)
        {
            x1 = x1/2;
            y1 = y1/2;
        }
        double joystickRightDistance = Math.pow(x2, 2) + Math.pow(y2, 2);
        if (joystickRightDistance < 0.9)
        {
            x2 = x2/2;
        }
        drive(x1,  y1 * -1, x2);

        if (gamepad1.right_trigger > 0.2) //in
        {
            intakeMotorLeft.setPower(1);
            intakeMotorRight.setPower(1);
        }
        else if (gamepad1.right_bumper) //out
        {
            intakeMotorLeft.setPower(-1);
            intakeMotorRight.setPower(-1);
        }
        else //stop when not in or out
        {
            intakeMotorLeft.setPower(0);
            intakeMotorRight.setPower(0);
        }

        if(gamepad1.left_bumper)
        {
            pullerLeft.setPosition(0);
            pullerRight.setPosition(0);
        }
        else if(gamepad1.left_trigger > 0.2)
        {
            pullerLeft.setPosition(1);
            pullerRight.setPosition(1);
        }

        if(gamepad2.right_stick_x > 0.5){
            if(wrist.getPosition() < 1){
                wrist.setPosition(wrist.getPosition() + 0.05);
            }
        }
        else if(gamepad2.right_stick_x < -0.5){
            if(wrist.getPosition() > 0){
                wrist.setPosition((wrist.getPosition() - 0.05));
            }
        }

        if(gamepad2.right_bumper)
        {
            finger.setPosition(0);
        }
        else if(gamepad2.right_trigger > 0.2)
        {
            finger.setPosition(1);
        }

        if(slideMotorLeft.getCurrentPosition() < -750 && gamepad2.left_stick_y < 0){
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }
        else if(slideMotorLeft.getCurrentPosition() > 0 && gamepad2.left_stick_y > 0){
            slideMotorLeft.setPower(0);
            slideMotorRight.setPower(0);
        }
        else if(gamepad2.left_stick_y < 0){
            if(slideMotorLeft.getCurrentPosition() < slideMotorRight.getCurrentPosition()){
                slideMotorLeft.setPower(gamepad2.left_stick_y / 2);
                slideMotorRight.setPower(gamepad2.left_stick_y);
            }
            else if(slideMotorRight.getCurrentPosition() < slideMotorLeft.getCurrentPosition()){
                slideMotorRight.setPower(gamepad2.left_stick_y / 2);
                slideMotorLeft.setPower(gamepad2.left_stick_y);
            }
            else {
                slideMotorLeft.setPower(gamepad2.left_stick_y);
                slideMotorRight.setPower(gamepad2.left_stick_y);
            }
        }
        else{
            if(slideMotorLeft.getCurrentPosition() < slideMotorRight.getCurrentPosition()){
                slideMotorLeft.setPower(gamepad2.left_stick_y / 3);
                slideMotorRight.setPower(gamepad2.left_stick_y / 6);
            }
            else if(slideMotorRight.getCurrentPosition() < slideMotorLeft.getCurrentPosition()){
                slideMotorRight.setPower(gamepad2.left_stick_y / 3);
                slideMotorLeft.setPower(gamepad2.left_stick_y / 6);
            }
            else {
                slideMotorLeft.setPower(gamepad2.left_stick_y / 3);
                slideMotorRight.setPower(gamepad2.left_stick_y / 3);
            }
            slideMotorLeft.setPower(gamepad2.left_stick_y / 3);
            slideMotorRight.setPower(gamepad2.left_stick_y / 3);
        }


        telemetry.addData("Slides: ", String.format("left: %5d, right: %5d", slideMotorLeft.getCurrentPosition(), slideMotorRight.getCurrentPosition()));
        telemetry.update();

    }
}

