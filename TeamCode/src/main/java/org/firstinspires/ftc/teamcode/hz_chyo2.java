/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
@TeleOp(name = "hz_chyo2")
public class hz_chyo2 extends LinearOpMode {

    private DcMotor RF = null;
    private DcMotor RB = null;
    private DcMotor LF = null;
    private DcMotor LB = null;

    // PinPoint Odometry
    private GobildaOdometryComputer odometryPod;
    private IMU imu;

    // Одометрия
    private double globalX = 0.0;
    private double globalY = 0.0;
    private double globalHeading = 0.0;

    // Предыдущие значения энкодеров
    private int prevXTicks = 0, prevYTicks = 0;
    private boolean firstOdometryUpdate = true;

    // Параметры одометрии
    private static final double WHEEL_DIAMETER = 48.0 / 1000.0; // 48mm в метрах
    private static final double ENCODER_TICKS_PER_REVOLUTION = 8192.0;

    double x = 0, y = 0, r = 0;

    @Override
    public void runOpMode() {
        // Инициализация двигателей
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);

        // Инициализация PinPoint одометрии
        initializeOdometry();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Odometry", "X: %.2fm, Y: %.2fm", globalX, globalY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(globalHeading));
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            // Обновление одометрии
            updateOdometry();

            // Управление
            y = gamepad1.left_stick_y;
            r = -gamepad1.left_trigger + gamepad1.right_trigger;
            move(y, r);

            // Боковое движение на правом стике
            if(Math.abs(gamepad1.right_stick_x) > 0.1) {
                float strafe = -gamepad1.right_stick_x;
                RB.setPower(strafe);
                RF.setPower(strafe);
                LF.setPower(strafe);
                LB.setPower(strafe);
            }

            // Движение по диагонали на D-pad
            if(gamepad1.dpad_up){
                x = 1;
                RB.setPower(x);
                RF.setPower(-x);
                LF.setPower(x);
                LB.setPower(-x);
            }
            if(gamepad1.dpad_down){
                x = 1;
                RB.setPower(-x);
                RF.setPower(x);
                LF.setPower(-x);
                LB.setPower(x);
            }

            // Сброс одометрии
            if(gamepad1.back && gamepad1.start) {
                resetOdometry();
            }

            // Вывод телеметрии одометрии
            displayOdometryTelemetry();
            telemetry.update();
        }
    }

    public void move(double y, double r) {
        RB.setPower(-y + r);
        RF.setPower(y - r);
        LF.setPower(-y - r);
        LB.setPower(y + r);
    }

    private void initializeOdometry() {
        try {
            // Инициализация Odometry Pod
            odometryPod = hardwareMap.get(GobildaOdometryComputer.class, "odometryPod");

            // Инициализация IMU
            imu = hardwareMap.get(IMU.class, "imu");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);
            imu.initialize(parameters);

            imu.resetYaw();

            // Сброс энкодеров
            odometryPod.resetEncoders();

            // Инициализация предыдущих значений
            prevXTicks = odometryPod.getLeftEncoder();
            prevYTicks = odometryPod.getRightEncoder();

            telemetry.addData("Odometry", "PinPoint initialized successfully");

        } catch (Exception e) {
            telemetry.addData("Odometry Error", "Failed to initialize: %s", e.getMessage());
        }
    }

    private void updateOdometry() {
        try {
            // Получение угла с IMU
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            globalHeading = -angles.getYaw(AngleUnit.RADIANS);

            // Чтение текущих значений энкодеров
            int currentXTicks = odometryPod.getLeftEncoder();   // Ось X
            int currentYTicks = odometryPod.getRightEncoder();  // Ось Y

            // Пропускаем первый вызов
            if (firstOdometryUpdate) {
                prevXTicks = currentXTicks;
                prevYTicks = currentYTicks;
                firstOdometryUpdate = false;
                return;
            }

            // Расчет изменений в тиках
            double deltaXTicks = currentXTicks - prevXTicks;
            double deltaYTicks = currentYTicks - prevYTicks;

            // Конвертация тиков в метры
            double deltaX = (deltaXTicks / ENCODER_TICKS_PER_REVOLUTION) * Math.PI * WHEEL_DIAMETER;
            double deltaY = (deltaYTicks / ENCODER_TICKS_PER_REVOLUTION) * Math.PI * WHEEL_DIAMETER;

            // Поворот в глобальную систему координат
            double rotatedX = deltaX * Math.cos(globalHeading) - deltaY * Math.sin(globalHeading);
            double rotatedY = deltaX * Math.sin(globalHeading) + deltaY * Math.cos(globalHeading);

            // Обновление глобальных координат
            globalX += rotatedX;
            globalY += rotatedY;

            // Сохранение текущих значений
            prevXTicks = currentXTicks;
            prevYTicks = currentYTicks;

        } catch (Exception e) {
            telemetry.addData("Odometry Error", "Update failed: %s", e.getMessage());
        }
    }

    private void resetOdometry() {
        globalX = 0.0;
        globalY = 0.0;
        globalHeading = 0.0;

        try {
            // Сброс Odometry Pod
            odometryPod.resetEncoders();

            // Сброс IMU
            imu.resetYaw();

            // Сброс предыдущих значений
            prevXTicks = 0;
            prevYTicks = 0;
            firstOdometryUpdate = true;

            telemetry.addData("Odometry", "Reset complete");

        } catch (Exception e) {
            telemetry.addData("Odometry Error", "Reset failed: %s", e.getMessage());
        }
    }

    private void displayOdometryTelemetry() {
        telemetry.addData("=== PINPOINT ODOMETRY ===", "");
        telemetry.addData("Position", "X: %.3fm, Y: %.3fm", globalX, globalY);
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(globalHeading));

        try {
            telemetry.addData("Encoders", "X: %d, Y: %d",
                    odometryPod.getLeftEncoder(),
                    odometryPod.getRightEncoder());
        } catch (Exception e) {
            telemetry.addData("Encoders", "Error reading");
        }

        telemetry.addLine();
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("Movement", "Left Stick - Forward/Back + Rotation");
        telemetry.addData("Strafe", "Right Stick X - Sideways");
        telemetry.addData("Diagonal", "D-pad Up/Down");
        telemetry.addData("Reset Odometry", "Back + Start");
        telemetry.addLine();
        telemetry.addData("Motor Powers", "RF: %.2f, RB: %.2f, LF: %.2f, LB: %.2f",
                RF.getPower(), RB.getPower(), LF.getPower(), LB.getPower());
    }
}*/