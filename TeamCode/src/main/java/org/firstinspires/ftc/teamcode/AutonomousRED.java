package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autoanus(RED)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
public class AutonomousRED extends LinearOpMode {
    //define
    private DcMotor RF = null;
    private DcMotor RB = null;
    private DcMotor LF = null;
    private DcMotor LB = null;

    private DcMotor NT = null;

    //private CRServo PDF = null;

    private DcMotor SM = null;


    int RFpos;
    int RBpos;
    int LFpos;
    int LBpos;

    @Override
    public void runOpMode() {
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");
        SM = hardwareMap.get(DcMotor.class, "sm");
        NT = hardwareMap.get(DcMotor.class, "nt");
        //PDF = hardwareMap.get(CRServo.class, "pdf");


        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        NT.setDirection(DcMotorSimple.Direction.REVERSE);

        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RBpos = 0;
        RFpos = 0;
        LBpos = 0;
        LFpos = 0;

        waitForStart();

        drive(-1000, 1000, 1000, -1000, 0.3);

        RF.setPower(0);
        LF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
        sleep(100);
        SM.setPower(-1);
        sleep(500);
        NT.setPower(-1);
        sleep(8000);
        drive(1000, 1000, 1000, 1000, 0.3);

    }

    private void drive(int RBtarg, int LBtarg, int RFtarg, int LFtarg, double speed) {
        RBpos += RBtarg;
        RFpos += RFtarg;
        LBpos += LBtarg;
        LFpos += LFtarg;

        RB.setTargetPosition(RBpos);
        RF.setTargetPosition(RFpos);
        LB.setTargetPosition(LBpos);
        LF.setTargetPosition(LFpos);

        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RF.setPower(speed);
        LF.setPower(speed);
        LB.setPower(speed);
        RB.setPower(speed);

        while (opModeIsActive() && RB.isBusy() && LB.isBusy() && RF.isBusy() && LF.isBusy()) {
            idle();
        }
    }
}