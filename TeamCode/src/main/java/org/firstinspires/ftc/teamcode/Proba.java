/*package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="TryTele")
public abstract class Proba extends OpMode {
    GoBildaPinpointDriver odo;

    private DcMotor RF;
    private DcMotor RB;
    private DcMotor LF;
    private DcMotor LB;

    @Override


    public void runOpMode(){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        LB = hardwareMap.get(DcMotor.class,"backleft");
        RB = hardwareMap.get(DcMotor.class,"backright");
        LF = hardwareMap.get(DcMotor.class,"frontleft");
        RF = hardwareMap.get(DcMotor.class,"frontright");


        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startingPosition= new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS,0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status","Initialized");
        //telemetry.addData("X offset", odo.getXOffset());
        //telemetry.addData("Y offset", odo.getYOffset());
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
*/