/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "hz_chyo21")
public class hz_chyo extends LinearOpMode {

    private DcMotor RF = null;
    private DcMotor RB = null;
    private DcMotor LF = null;
    private DcMotor LB = null;
    // Servo Ser;
    double x = 0, y = 0, r = 0;



    @Override
    public void runOpMode() {
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive()) {
//            x = -gamepad1.left_stick_x;
            y = gamepad1.left_stick_y;
            r = -gamepad1.left_trigger + gamepad1.right_trigger;
            move( y, r);

            if(gamepad1.right_stick_x > 0) {
                float x = -gamepad1.right_stick_x;
                RB.setPower(x);
                RF.setPower(x);
                LF.setPower(x);
                LB.setPower(x);
            }
            if(gamepad1.right_stick_x < 0) {
                float x = -gamepad1.right_stick_x;
                RB.setPower(x);
                RF.setPower(x);
                LF.setPower(x);
                LB.setPower(x);
            }


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


        }
    }
    public void move( double y, double r) {
        RB.setPower(-y+r);
        RF.setPower(y-r);
        LF.setPower(-y-r);
        LB.setPower(y+r);
    }

}
*/
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "hz_chyo2")
public class hz_chyo extends LinearOpMode {
    GoBildaPinpointDriver odo;

    private DcMotor RF;
    private DcMotor RB;
    private DcMotor LF;
    private DcMotor LB;

    @Override


    public void runOpMode(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        LB = hardwareMap.get(DcMotor.class,"lb");
        RB = hardwareMap.get(DcMotor.class,"rb");
        LF = hardwareMap.get(DcMotor.class,"lf");
        RF = hardwareMap.get(DcMotor.class,"rf");


        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startingPosition= new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS,0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status","Initialized");
        /*telemetry.addData("X offset", odo.getXOffset());
        /telemetry.addData("Y offset", odo.getYOffset());*/
        telemetry.addData("Devic Version Number:", odo.getDeviceVersion());
        telemetry.addData("Devic Scalar:", odo.getYawScalar());
        telemetry.update();
    }



    public void moveRobot(){
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.left_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        LF.setPower(newWheelSpeeds[0]);
        RF.setPower(newWheelSpeeds[1]);
        LB.setPower(newWheelSpeeds[2]);
        RB.setPower(newWheelSpeeds[3]);
        telemetry.addData("Robot XPos:", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos:", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading:", heading);
        telemetry.addData("Forward Speed:", globalForward);
        telemetry.addData("Strafe Speed:", globalStrafe);


        telemetry.addData("Forward Speed", globalForward);
        telemetry.addData("Strafe Speed:", globalStrafe);

    }



}

