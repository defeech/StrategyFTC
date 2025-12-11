package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;

@Autonomous(name = "AutonomousNiceOdo")
public class AutonomousNiceOdo extends LinearOpMode {
    //define
    private final double POS_ERROR_TOLERANCE = 0.01;
    private final double HEADING_ERROR_TOLERANCE = 0.01;

    private final double MAX_MOTOR_CURRENT = 9.5;
    private final double DEAD_WHEEL_RADIUS_MM = 16;
    //PID controls

    //For Drive Movement
    private static double kDP = 1;
    private static double kDI = 0;
    private static double kDD = 0;
    private static double errorSumDX = 0;
    private static double errorSumDY = 0;
    private static double errorSumRangeD = 5;

    //For Turn Movement
    private static double kTP = 0.25;
    private static double kTI = 0.01;
    private static double kTD = 0.02;
    private static double errorSumT = 0;
    private static double errorSumRangeT = 25;



    //Drive motor names
    private String leftFrontName = "lf";
    private String leftBackName = "lb";
    private String rightFrontName = "rf";
    private String rightBackName = "rb";

    //Drive motor vars
    private static DcMotorEx LF;
    private static DcMotorEx LB;
    private static DcMotorEx RF;
    private static DcMotorEx RB;
    private CRServo PDF;
    private static DcMotorEx SM;



    private static int leftFrontNum;
    private static int leftBackNum;
    private static int rightFrontNum;
    private static int rightBackNum;

    private static DcMotorControllerEx motorControllerEx;

    private static double[] motorCurrents = new double[4];

    //Gobilda Pinpoint
    private String odoName = "odo";
    private GoBildaPinpointDriver odo;

    //Pinpoint offsets from Center in mm and encoder Direction
    //Left is +X, Front is +Y
    private double xoffset = 100;
    private double yoffset = 100;

    private GoBildaPinpointDriver.EncoderDirection xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    //Dead Wheel Type

    private GoBildaPinpointDriver.GoBildaOdometryPods encoderPod = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    //LinearOpmode

    // private LinearOpMode opMode;


    //Start Positions as {y,x} format and add to the master list for poses

    private double[] pos1 = new double[] {0,0, 0};
    private double[] pos2 = new double[] {8,36, 0 };


    private double[][] masterStartPoses = new double[][] { pos1, pos2};


    //Background Varables

    public static double timeLimit = 0;
    public static boolean outInfo = true;

    @Override
    public void runOpMode() {

        LF = hardwareMap.get(DcMotorEx.class, leftFrontName);
        LB = hardwareMap.get(DcMotorEx.class, leftBackName);
        RF = hardwareMap.get(DcMotorEx.class, rightFrontName);
        RB = hardwareMap.get(DcMotorEx.class, rightBackName);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontNum = LF.getPortNumber();
        leftBackNum = LB.getPortNumber();
        rightFrontNum = RF.getPortNumber();
        rightBackNum = RB.getPortNumber();

        motorControllerEx = (DcMotorControllerEx)(LF.getController());

        motorCurrents = getMotorCurrents();

        motorControllerEx.setMotorCurrentAlert(leftFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(leftBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightFrontNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);
        motorControllerEx.setMotorCurrentAlert(rightBackNum, MAX_MOTOR_CURRENT, CurrentUnit.AMPS);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, odoName);

        //Pinpoint offsets from Center in mm
        //Left is +X, Front is +Y
        odo.setOffsets(xoffset, yoffset);
        odo.setEncoderDirections(xDirection, yDirection);
        odo.setEncoderResolution(encoderPod);

        odo.recalibrateIMU();
        sleep(50);
        odo.resetPosAndIMU();
        sleep(50);

        odo.setPosition(new Pose2D(DistanceUnit.CM, masterStartPoses[0][1],masterStartPoses[0][0], AngleUnit.DEGREES, masterStartPoses[0][0]));
        sleep(10);

        waitForStart();
        resetRuntime();

        while(opModeIsActive()){

            forward(1);

            goToHeading(90);

            forward(1);


            telemetry.addData("LF Direction: ", LF.getDirection());
            telemetry.addData("LB Direction: ", LB.getDirection());
            telemetry.addData("RF Direction: ", RF.getDirection());
            telemetry.addData("RB Direction: ", RB.getDirection());
            telemetry.update();
        }
    }
    public static void drive(double RFPower, double RBPower, double LBPower, double LFPower) {

        RF.setPower(RFPower);
        RB.setPower(RBPower);
        LB.setPower(LBPower);
        LF.setPower(LFPower);
    }

    public GoBildaPinpointDriver getPinPoint(){return odo; }

    public DcMotorEx getMotor(int num){
        switch (num){
            case 0:
                return LF;
            case 1:
                return LB;
            case 2:
                return RF;
            case 3:
                return RB;

        }
        return LF;
    }

    public Pose2D getPos(){return  odo.getPosition(); }
    public double getX(){return  odo.getPosition().getX(DistanceUnit.CM); }
    public double getY(){return  odo.getPosition().getY(DistanceUnit.CM); }

    //getHeading outputs 0-360
    public double getHeading(){
        double rawHeading = odo.getPosition().getHeading(AngleUnit.DEGREES);
        return  rawHeading + 180;
    }
    //getHeadingNorm outputs -180-180
    public double getHeadingNorm(){
        return odo.getPosition().getHeading(AngleUnit.DEGREES);
    }
    //getHeadingUnNorm outputs -inf-inf
    public double getHeadingUnNorm(){
        return odo.getHeading();
    }


    public double getAngleToGo(double targetHeading){
        targetHeading = Math.abs(targetHeading) % 360;

        double currentHeading = getHeading();
        double angleTogo = targetHeading - currentHeading;
        currentHeading =  getHeading();

        angleTogo = targetHeading - currentHeading;

        if(Math.abs(angleTogo) > 180) {
            if (currentHeading < 180) {
                angleTogo = -((currentHeading) + (360 - targetHeading));
            } else {
                angleTogo = (targetHeading + (360 - currentHeading));
            }
        }
        return angleTogo;
    }

    public static double[] getMotorCurrents(){
        double currentLF = motorControllerEx.getMotorCurrent(leftFrontNum, CurrentUnit.AMPS);
        double currentLB = motorControllerEx.getMotorCurrent(leftBackNum, CurrentUnit.AMPS);
        double currentRF = motorControllerEx.getMotorCurrent(rightFrontNum, CurrentUnit.AMPS);
        double currentRB = motorControllerEx.getMotorCurrent(rightBackNum, CurrentUnit.AMPS);
        return new double[] {currentRF, currentRB, currentLB, currentLF};
    }

    public static ArrayList<Boolean> isMotorsOver(){
        boolean overCurrentLF = motorControllerEx.isMotorOverCurrent(leftFrontNum);
        boolean overCurrentLB = motorControllerEx.isMotorOverCurrent(leftBackNum);
        boolean overCurrentRF = motorControllerEx.isMotorOverCurrent(rightFrontNum);
        boolean overCurrentRB = motorControllerEx.isMotorOverCurrent(rightBackNum);

        ArrayList<Boolean> list = new ArrayList<>();

        list.add(overCurrentLF);
        list.add(overCurrentLB);
        list.add(overCurrentRF);
        list.add(overCurrentRB);

        return list;
    }

    public void setPID(double kP, double kI, double kD, int PIDNum){
        switch (PIDNum){
            case 0:
                kDP = kP;
                kDI = kI;
                kDD = kD;
                break;
            case 1:
                kTP = kP;
                kTI = kI;
                kTD = kD;
                break;

        }
    }


    //Set Limit to 0 if you don't want a time limit
    //Default is 0
    public static void setTimeLimit(double time){
        timeLimit = time;
    }

    public boolean checkTime(double startTime, double currentTime){
        if((currentTime - startTime) >= timeLimit){
            return true;
        }else {
            return false;
        }
    }

    public void setOutputInfo(boolean val){
        outInfo = val;
    }

    public void outputInfo(){
        if(outInfo){
            telemetry.addData("X pos: ", getX());
            telemetry.addData("Y pos: ", getY());
            telemetry.addData("Heading: ", getHeading());
            telemetry.addData("Heading Norm: ", getHeadingNorm());
            telemetry.addData("Heading UnNorm: ", odo.getHeading());
            //telemetry.addData("LF Direction: ", LF.getDirection());
            //telemetry.addData("LB Direction: ", LB.getDirection());
            //telemetry.addData("RF Direction: ", RF.getDirection());
            //telemetry.addData("RB Direction: ", RB.getDirection());
            telemetry.update();
        }
    }

    public double turnSlope(double targetHeading, double startHeading, double currentDist, double startDist){
        return ((startHeading-targetHeading)/startDist) * (currentDist - startDist) + startHeading;
    }

    public double CMToTicks(double dist){
        return (dist * 10 * 2000)/ (DEAD_WHEEL_RADIUS_MM * 2 * Math.PI);
    }


    //PIDs

    private static double movePID(double error, String axis){
        double output = error * kDP - error * kDD;
        if(Math.abs(error) <= errorSumRangeD){
            if(axis.toLowerCase().charAt(0) == 'y'){
                errorSumDY+= error;
                output += errorSumDY*kDI;
            }
            if(axis.toLowerCase().charAt(0) == 'x'){
                errorSumDX+= error;
                output += errorSumDX*kDI;
            }
        }
        return output;
    }

    private static double turnPID(double error){
        double output = error * kTP - error * kTD + errorSumT*kTI;
        if(Math.abs(error) <= errorSumRangeT){
            errorSumT += error;
        }
        return output;
    }


    //Movement

    public void forward(double distance){
        double power = movePID(distance, "x");
        double startPose = odo.getEncoderX();
        double targetPose = CMToTicks(distance) + startPose;
        while(targetPose - startPose > 5) {
            startPose = odo.getEncoderX();
            power = movePID(targetPose - startPose, "x");
            drive(power, power, power, power);
        }
    }
    public void goToPointConstantHeading(double targetX, double targetY){
        odo.update();

        errorSumDX = 0;
        errorSumDY = 0;
        errorSumT = 0;

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);


        double startHeading = getHeading();
        double turn = turnPID(startHeading);

        double startTime = time;


        while(!checkTime(startTime,time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(startHeading)) > HEADING_ERROR_TOLERANCE)
                && opModeIsActive()) {

            odo.update();
            outputInfo();

            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());

            double angleToGo = getAngleToGo(startHeading);

            double v1 = 0;// LF
            double v2 = 0; // RF
            double v3 = 0; // LB
            double v4 = 0; // RB


            double y = -movePID(targetXDist,"y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist,"x");
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            rotX = rotX * 1.1;  // Counteract impeRFect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;



            drive(v2,v4,v3,v1);
        }
        sleep(50);
    }

    public void goToPointLinear(double targetX, double targetY, double targetHeading){
        odo.update();

        double targetXDist = targetX - getX();
        double targetYDist = targetY - getY();
        double totalDist = Math.hypot(targetXDist, targetYDist);

        double startDist = totalDist;

        double startHeading = getHeading();
        double headingdist = targetHeading - startHeading;
        double currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);


        double startTime = time;


        while(!checkTime(startTime,time) && (Math.abs(targetXDist) > POS_ERROR_TOLERANCE
                ||  Math.abs(targetYDist) > POS_ERROR_TOLERANCE || Math.abs(getAngleToGo(currentTargetHead)) > HEADING_ERROR_TOLERANCE)
                && opModeIsActive()) {

            odo.update();
            outputInfo();
            ;

            targetXDist = targetX - getX();
            targetYDist = targetY - getY();
            totalDist = Math.hypot(targetXDist, targetYDist);

            double currentHeadingRad = Math.toRadians(getHeadingNorm());
            currentTargetHead = turnSlope(targetHeading, startHeading, totalDist, startDist);
            double angleToGo = getAngleToGo(currentTargetHead);

            double v1 = 0;// LF
            double v2 = 0; // RF
            double v3 = 0; // LB
            double v4 = 0; // RB


            double y = -movePID(targetXDist, "y"); // Remember, Y stick value is reversed
            double x = movePID(targetYDist, "x");
            double rx = turnPID(angleToGo);
            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            rotX = rotX * 1.1;  // Counteract impeRFect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            v1 = (rotY + rotX + rx) / denominator;
            v3 = (rotY - rotX + rx) / denominator;
            v2 = (rotY - rotX - rx) / denominator;
            v4 = (rotY + rotX - rx) / denominator;



            drive(v2,v4,v3,v1);
        }
        sleep(50);
    }

    public void goToHeading(double heading){
        heading = Math.abs(heading) % 360;
        odo.update();
        double currentHeading = getHeading();
        double angleToGo = getAngleToGo(heading);
        double power;
        while(angleToGo > HEADING_ERROR_TOLERANCE){
            outputInfo();
            power = turnPID(angleToGo);
            drive(power, power,-power,-power);
        }
        sleep(50);
    }

}