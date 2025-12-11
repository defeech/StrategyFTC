package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class AutonomousEasyOdo extends LinearOpMode {
    private DcMotor RF = null;
    private DcMotor RB = null;
    private DcMotor LF = null;
    private DcMotor LB = null;
    private DcMotor SM = null;
    private DcMotor NT = null;
    private CRServo PDF = null;
    private GoBildaPinpointDriver odo;



    private final double DEAD_WHEEL_RADIUS_MM = 16;

    double power = 1;

    @Override
    public void runOpMode() {
        RF = hardwareMap.get(DcMotor.class, "rf");
        RB = hardwareMap.get(DcMotor.class, "rb");
        LF = hardwareMap.get(DcMotor.class, "lf");
        LB = hardwareMap.get(DcMotor.class, "lb");
        SM = hardwareMap.get(DcMotor.class, "sm");
        NT = hardwareMap.get(DcMotor.class, "nt");

        PDF = hardwareMap.get(CRServo.class, "pdf");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        //RF.setDirection(DcMotorSimple.Direction.REVERSE);
        //LF.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();
        sleep(50);
        odo.resetPosAndIMU();
        sleep(50);

        waitForStart();
        drive(5, 0, 0, 0.5);
        drive(0, 5, 0, 0.5);
    }

    public double CMToTicks(double dist){
        return (dist * 10 * 2000)/ (DEAD_WHEEL_RADIUS_MM * 2 * Math.PI);
    }

    private void drive(double XtargCM, double YtargCM, double rotate, double power) {

        double Xtarg = CMToTicks(XtargCM);
        double Ytarg = CMToTicks(YtargCM);

        double tolerance = 50;

        double CurrentEncX = odo.getEncoderX();
        double CurrentEncY = odo.getEncoderY();

        if(YtargCM == 0) {
            while (opModeIsActive() && Math.abs(Xtarg - CurrentEncX) > tolerance) {
                CurrentEncX = odo.getEncoderX();
                driveTTX(XtargCM, power);
            }
        }
        else{
            while (opModeIsActive() && Math.abs(Ytarg - CurrentEncY) > tolerance) {
                CurrentEncY = odo.getEncoderY();
                driveTTY(YtargCM, power);
            }
        }

        stopM();
        sleep(100);

    }

    public void driveTTX(double posX, double power) {
        if(posX > 0){
            RF.setPower(power);
            LF.setPower(-power);
            RB.setPower(-power);
            LB.setPower(power);
        }
        if(posX < 0){
            RF.setPower(-power);
            LF.setPower(power);
            RB.setPower(power);
            LB.setPower(-power);
        }
    }

    public void driveTTY(double posY, double power){
        if(posY > 0){
            RF.setPower(power);
            LF.setPower(power);
            RB.setPower(power);
            LB.setPower(power);
        }
        if(posY < 0){
            RF.setPower(-power);
            LF.setPower(-power);
            RB.setPower(-power);
            LB.setPower(-power);
        }
    }

    public void stopM(){
        RF.setPower(0);
        LF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
    }
}

