

package org.firstinspires.ftc.teamcode.drive.teleop;

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

    private Servo claw = null;
    private Servo wrist = null;

    double clawPosition = 0;

    public static double armDownSpeed = 0.2;
    public static double armUpSpeed = -0.3;

    public static int targetPosition = 0;

    // Motors at the beginning of the arm
    private DcMotorEx belt = null; // Belt motor
    private DcMotorEx belt2 = null;


    //double clawOffset  = 0.0;
    //final double CLAW_SPEED  = 0.2;
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

        //wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        RF.setDirection(DcMotorEx.Direction.FORWARD);
        RB.setDirection(DcMotorEx.Direction.FORWARD);
        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.REVERSE);

        belt2.setDirection(DcMotorEx.Direction.REVERSE);

        belt.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        belt2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        belt.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        belt2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        belt2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        claw = hardwareMap.get(Servo.class, "claw");
        claw.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);

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


        claw.setPosition(0);
        wrist.setPosition(0.4);

        targetPosition = 0;
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

        double powerOffset = 0.6;
        LF.setPower(frontLeftPower * powerOffset);
        LB.setPower(backLeftPower * powerOffset);
        RF.setPower(frontRightPower * powerOffset);
        RB.setPower(backRightPower * powerOffset);

        if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            targetPosition = targetPosition + 1;

            belt.setTargetPosition(targetPosition);
            belt2.setTargetPosition(targetPosition);

            belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            belt.setPower(armUpSpeed);
            belt2.setPower(armUpSpeed);




        }

        if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            targetPosition = targetPosition - 1;

            belt.setTargetPosition(targetPosition);
            belt2.setTargetPosition(targetPosition);

            belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            belt.setPower(armDownSpeed);
            belt2.setPower(armDownSpeed);

        }


        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            //clawOffset += CLAW_SPEED;
            claw.setPosition(0.4);
            wrist.setPosition(0.4);
            /*telemetry.addData("Status: ", "Opening");
            telemetry.addData("Position: ", claw.getPosition());*/


        }

        if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            //clawOffset -= CLAW_SPEED;
            claw.setPosition(0.15);
            wrist.setPosition(0.4);
            /*telemetry.addData("Status: ", "Closing");
            telemetry.addData("Position: ", claw.getPosition());*/


        }

        if (gamepad2.dpad_down) {
            belt.setTargetPosition(0);
            belt2.setTargetPosition(0);

            belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            belt.setPower(armDownSpeed);
            belt2.setPower(armDownSpeed);

            wrist.setPosition(0.4);
        }

        if (gamepad2.dpad_up) {
            belt.setTargetPosition(590);
            belt2.setTargetPosition(590);

            belt.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            belt2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            belt.setPower(armUpSpeed);
            belt2.setPower(armUpSpeed);

            wrist.setPosition(0.46);
        }

        /*if (gamepad2.y) {
            wrist.setPosition(0.7);
        }

        if (gamepad2.x) {
            wrist.setPosition(0.425);
        }*/

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
