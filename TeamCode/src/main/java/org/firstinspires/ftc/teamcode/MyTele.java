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
    Servo pullerLeft;
    Servo pullerRight;

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
            telemetry.addData("approachingSpeedLeft: ", "%f", approachingSpeedLeft);
            telemetry.addData("approachingSpeedRight: ", "%f", approachingSpeedRight);
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
    }
}

