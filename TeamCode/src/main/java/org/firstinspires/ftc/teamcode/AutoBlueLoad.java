package org.firstinspires.ftc.teamcode;

        import android.util.Log;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
        import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
        import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
        import org.firstinspires.ftc.teamcode.Library.ImageNavigation;
        import org.firstinspires.ftc.teamcode.Library.MyBoschIMU;

@Autonomous(name = "AutoBlueLoad", group = "none")
public class AutoBlueLoad extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        Strafe(0.6F, 5F, Direction.RIGHT);

        sleep(600);

        resetAllEncoders();

        if (!StrafeToImage(0.55F, imageNavigation.stoneTarget, this, 8)) {
            Drive(0.4F, 8F, Direction.BACKWARD);

            resetAllEncoders();
            sleep(600);
            if (!StrafeToImage(0.55F, imageNavigation.stoneTarget, this, 8)) {
                Drive(0.4F, 6F, Direction.FORWARD);
                StrafeUntilDistance(0.5F, Direction.RIGHT, 5, primaryAngle, imu);

                int i = 0;

                while (!detectSkystoneByColor() && i < 2) {
                    Drive(0.4f, 7, Direction.BACKWARD);
                    i++;
                    telemetry.update();
                    sleep(350);
                }
                skystonePosition = i + 1;
                telemetry.update();
            }
            else {
                double distance = getLateralMovement();
                if (distance < -4) {
                    skystonePosition = 3;
                } else {
                    skystonePosition = 2;
                }
            }
        } else {
            double distance = getLateralMovement();
            if (distance < -4) {
                skystonePosition = 2;
            } else {
                skystonePosition = 1;
            }
        }

        Log.i("[phoenix]", String.format("skystonePosition = %d", skystonePosition));

        float distanceX = 6;
//        distanceX = distanceX - getSkystoneXPosition(imageNavigation.stoneTarget);
        if(lastKnownPosition.translation != null)
            distanceX = distanceX - lastKnownPosition.translation.get(1) / 25.4f;
        telemetry.addData("DistanceX: ", "%f", distanceX);
        telemetry.update();

        Drive(0.4F, distanceX, Direction.BACKWARD);
        sleep(100);
        Turn(0.4F, 45, Direction.CLOCKWISE, imu, this);
        sleep(100);
        intakeMotorLeft.setPower(1);
        intakeMotorRight.setPower(1);

        float distanceZ = 7;

        if(lastKnownPosition.translation != null)
            distanceZ = (distanceZ + (Math.abs(lastKnownPosition.translation.get(2)) / 25.4f - 3f)) * (float)Math.sqrt(2);
        Drive(0.4F, distanceZ, Direction.FORWARD);

        sleep(300);
        intakeMotorLeft.setPower(0);
        intakeMotorRight.setPower(0);

        Drive(0.9F, distanceZ + 1, Direction.BACKWARD);

        imu.resetAndStart(Direction.COUNTERCLOCKWISE);

        float angleToBuild = primaryAngle - imu.getAngularOrientation().firstAngle;

        Log.i("[phoenix]", String.format("primary = %d; angleToBuild = %f10.2", primaryAngle, angleToBuild ));
        telemetry.addData("cac result", String.format("primary = %d; angleToBuild = %f10.2", primaryAngle, angleToBuild ));
        telemetry.update();

        Turn(0.4F, (int)angleToBuild, Direction.COUNTERCLOCKWISE, imu, this);

        sleep(100);

        //driving towards build zone
        Log.i("[phoenix:skystonePos]", String.format("skystonePosition=%d", skystonePosition));
        sleep(100);
        double distanceToBuildZone = 50 + 8 * (skystonePosition - 1);
        Drive(0.6F, (float)distanceToBuildZone, Direction.FORWARD);

        releaseStone();

        sleep(100);
        Turn(0.6f,179, Direction.CLOCKWISE, imu, this);
        Drive(0.7F, 43F, Direction.FORWARD);

        sleep(100);
        OpenGLMatrix coordinates = imageNavigation.getRobotLocation();
        VectorF vector = coordinates.getTranslation();

        double dy = 8;

        double skystoneY = 48 + (skystonePosition - 1) * 8;

        if (coordinates != null) {
            Log.i("[phoenix:nav]", String.format("x=%f10.2; y=%f10.2; z=%f10.2", vector.get(0), vector.get(1), vector.get(2)));

            dy = (Math.abs(vector.get(1) / 25.4) - 24) - 7; // distance between robot and stone
            double dx = Math.abs(vector.get(0) / 25.4) - (skystoneY - dy);

            Log.i("[phoenix:nav]", String.format("dx=%f10.2; dy=%f10.2", dx, dy));

            if (dx > 1)
                Drive(0.4f, (float) Math.abs(dx), Direction.BACKWARD);
            else if (dx < -1)
                Drive(0.4f, (float) Math.abs(dx), Direction.FORWARD);

        }
        else {
            Log.i("[phoenix:nav]", "Can't see Image");
        }
        sleep(100);

        Turn(0.4f, 45, Direction.COUNTERCLOCKWISE, imu, this);
        intakeMotorLeft.setPower(1);
        intakeMotorRight.setPower(1);

        Drive(0.5f, (float) Math.abs(dy)* (float) Math.sqrt(2), Direction.FORWARD);
        sleep(300);
        intakeMotorRight.setPower(0);
        intakeMotorLeft.setPower(0);
        float angleToStone = Math.abs((180 - Math.abs(imu.getAngularOrientation().firstAngle)));
        Log.i("[phoenix]", String.format("angleToStone = %f10.2", angleToStone));
        Drive(0.7f, 12/(float)Math.sin(Math.toRadians(angleToStone)), Direction.BACKWARD);

        imu.resetAndStart(Direction.COUNTERCLOCKWISE);
        angleToBuild = primaryAngle + imu.getAngularOrientation().firstAngle;

        Log.i("[phoenix]", String.format("primary = %d; angleToBuild = %f10.2", primaryAngle, angleToBuild ));
        telemetry.addData("cac result", String.format("primary = %d; angleToBuild = %f10.2", primaryAngle, angleToBuild ));
        telemetry.update();

        Turn(0.4F, 180 + (int)angleToBuild, Direction.CLOCKWISE, imu, this);

        distanceToBuildZone = (48 + (skystonePosition -1) * 8) + 12 - 12/Math.tan(Math.toRadians(angleToStone));
        Drive(0.7F, (float)distanceToBuildZone, Direction.BACKWARD);
        Turn(0.4f, 90, Direction.COUNTERCLOCKWISE, imu, this);

        releaseStone();

        Strafe(0.5f, 8, Direction.RIGHT);
    }
}
