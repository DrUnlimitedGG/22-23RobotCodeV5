

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name="MecanumDrive", group="TeleOp")
public class  MecanumDrive extends OpMode
{
    // Drivetrain motors
    private DcMotorEx LF = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx LB = null;

    public static double armDownSpeed = 0.1;
    public static double armUpSpeed = -0.15;

    public static int targetPosition = 0;

    // Motors at the beginning of the arm
    private DcMotorEx belt = null; // Belt motor
    private DcMotorEx belt2 = null;

    // Motors at the end of the arm
    private Servo wrist = null;
    private Servo claw = null;

    double clawOffset  = 0.0;
    final double CLAW_SPEED  = 0.2;
    /*

     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt2 = hardwareMap.get(DcMotorEx.class, "belt2");

        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        RF.setDirection(DcMotorEx.Direction.FORWARD);
        RB.setDirection(DcMotorEx.Direction.FORWARD);
        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        belt2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        belt2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override

    public void start() {

        RF.setPower(0);
        RB.setPower(0);
        LB.setPower(0);
        LF.setPower(0);

        //claw.setPosition(0.22);
        //wrist.setPosition(0.5);
    }

    /*
     * eqwCode to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        float spin_power = 0.30f; // The spin power
        boolean dpad = false;     // A boolean used to indicate whether the dpad has been pressed used for the changing of spin power

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        double powerOffset = 0.1;
        LF.setPower(frontLeftPower * powerOffset);
        LB.setPower(backLeftPower * powerOffset);
        RF.setPower(frontRightPower * powerOffset);
        RB.setPower(backRightPower * powerOffset);

        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            targetPosition = targetPosition - 2;

            belt.setTargetPosition(targetPosition);
            belt2.setTargetPosition(targetPosition);

            belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            belt.setPower(armUpSpeed);
            belt2.setPower(armUpSpeed);




        }

        if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            targetPosition = targetPosition + 2;

            belt.setTargetPosition(targetPosition);
            belt2.setTargetPosition(targetPosition);

            belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            belt.setPower(armDownSpeed);
            belt2.setPower(armDownSpeed);

            telemetry.addData("yo we moving DOWN ", belt.getCurrentPosition());
            telemetry.addData("speed ", belt.getPower());
        }


       if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            //clawOffset += CLAW_SPEED;
           claw.setPosition(0.22);

        } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            //clawOffset -= CLAW_SPEED;
           claw.setPosition(0.5);
        }

        telemetry.addData("Belt 1 Power: ", belt.getVelocity());
       telemetry.addData("Belt 2 Power: ", belt2.getVelocity());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    /* public void extendArm(int targetPosition) {
        belt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        belt2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        belt.setTargetPosition(targetPosition);
        belt2.setTargetPosition(targetPosition);

        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        belt.setVelocity(armUpSpeed);
        belt2.setVelocity(armUpSpeed);

        if (!belt.isBusy() && !belt2.isBusy()) {
            belt.setPower(0);
            belt2.setPower(0);

        }

        telemetry.addData("Belt moving up: ", belt.getCurrentPosition());

    }

    public void retractArm(int targetPosition) {
        belt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        belt2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        belt.setTargetPosition(targetPosition);
        belt2.setTargetPosition(targetPosition);

        belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        belt.setVelocity(armDownSpeed);
        belt2.setVelocity(armDownSpeed);

        if ((targetPosition - belt.getCurrentPosition()) <= 50 && (targetPosition - belt2.getCurrentPosition()) <= 50) {
            belt.setVelocity(0);
            belt2.setVelocity(0);

        }

        telemetry.addData("Belt moving down: ", belt.getCurrentPosition());

    } */

}
