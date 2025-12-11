package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="GoBILDA Pinpoint Autonomous", group="Autonomous")
@Disabled
public class GoBildaPinpointAutonomous extends LinearOpMode {

    private GoBildaPinpointDriver odo;
    private DcMotor RF = null;
    private DcMotor RB = null;
    private DcMotor LF = null;
    private DcMotor LB = null;

    private ElapsedTime runtime = new ElapsedTime();

    private static final double POSITION_TOLERANCE_MM = 5.0;
    private static final double HEADING_TOLERANCE_DEG = 2.0;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            driveToPosition(200, 0, 0, 1.0);    // Движение вперед 500mm
            turnToHeading(90, 1.0);              // Поворот на 90 градусов
            driveToPosition(200, 200, 90, 2.0);  // Движение по диагонали
            turnToHeading(0, 1.0);               // Возврат к исходному углу
            driveToPosition(0, 0, 0, 2.0);       // Возврат в стартовую позицию

            stopRobot();
        }
    }



    private void driveToPosition(double targetX, double targetY, double targetHeading, double timeoutS) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            odo.update();
            Pose2D currentPos = odo.getPosition();

            double currentX = currentPos.getX(DistanceUnit.MM);
            double currentY = currentPos.getY(DistanceUnit.MM);
            double currentHeading = currentPos.getHeading(AngleUnit.DEGREES);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double errorHeading = normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(errorX) < POSITION_TOLERANCE_MM &&
                    Math.abs(errorY) < POSITION_TOLERANCE_MM &&
                    Math.abs(errorHeading) < HEADING_TOLERANCE_DEG) {
                break;
            }

            double kpLinear = 0.01;
            double kpAngular = 0.02;

            double powerX = errorX * kpLinear;
            double powerY = errorY * kpLinear;
            double powerTurn = errorHeading * kpAngular;

            powerX = clipPower(powerX);
            powerY = clipPower(powerY);
            powerTurn = clipPower(powerTurn);

            double frontLeftPower = powerY + powerX + powerTurn;
            double frontRightPower = powerY - powerX - powerTurn;
            double backLeftPower = powerY - powerX + powerTurn;
            double backRightPower = powerY + powerX - powerTurn;

            setMotorPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

            telemetry.addData("Target", "X:%.1f, Y:%.1f, H:%.1f", targetX, targetY, targetHeading);
            telemetry.addData("Current", "X:%.1f, Y:%.1f, H:%.1f", currentX, currentY, currentHeading);
            telemetry.addData("Errors", "X:%.1f, Y:%.1f, H:%.1f", errorX, errorY, errorHeading);
            telemetry.update();
        }

        stopRobot();
    }

    private void turnToHeading(double targetHeading, double timeoutS) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            odo.update();
            double currentHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
            double error = normalizeAngle(targetHeading - currentHeading);

            if (Math.abs(error) < HEADING_TOLERANCE_DEG) {
                break;
            }

            double turnPower = error * 0.03;
            turnPower = clipPower(turnPower);

            setMotorPowers(turnPower, -turnPower, turnPower, -turnPower);

            telemetry.addData("Turn Target", "%.1f°", targetHeading);
            telemetry.addData("Current Heading", "%.1f°", currentHeading);
            telemetry.addData("Turn Error", "%.1f°", error);
            telemetry.update();
        }

        stopRobot();
    }

    private void driveDistance(double distanceMM, double speed, double timeoutS) {
        odo.update();
        Pose2D startPos = odo.getPosition();
        double startX = startPos.getX(DistanceUnit.MM);
        double startY = startPos.getY(DistanceUnit.MM);

        double currentHeading = startPos.getHeading(AngleUnit.RADIANS);
        double targetX = startX + distanceMM * Math.cos(currentHeading);
        double targetY = startY + distanceMM * Math.sin(currentHeading);

        driveToPosition(targetX, targetY, startPos.getHeading(AngleUnit.DEGREES), timeoutS);
    }


    /* private void strafeDistance(double distanceMM, double speed, double timeoutS) {
        odo.update();
        Pose2D startPos = odo.getPosition();
        double startX = startPos.getX(DistanceUnit.MM);
        double startY = startPos.getY(DistanceUnit.MM);

        double currentHeading = startPos.getHeading(AngleUnit.RADIANS);
        double targetX = startX + distanceMM * Math.cos(currentHeading + Math.PI/2);
        double targetY = startY + distanceMM * Math.sin(currentHeading + Math.PI/2);

        driveToPosition(targetX, targetY, startPos.getHeading(AngleUnit.DEGREES), timeoutS);
    } */


    private void setMotorModes(DcMotor.RunMode mode) {
        RF.setMode(mode);
        RB.setMode(mode);
        LF.setMode(mode);
        LB.setMode(mode);
    }

    private void setMotorPowers(double lf, double rf, double lb, double rb) {
        LF.setPower(lf);
        RF.setPower(rf);
        LB.setPower(lb);
        RB.setPower(rb);
    }

    private void stopRobot() {
        setMotorPowers(0, 0, 0, 0);
    }

    private double clipPower(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }

    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}