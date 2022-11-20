

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MecanumDrive", group="TeleOp")
public class MecanumDrive extends OpMode
{
    // Drivetrain motors
    private DcMotorEx LF = null;
    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx LB = null;

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

        claw.setPosition(0.22);
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
            belt.setPower(-0.001); // Reverse because direction flipped
            belt2.setPower(-0.001);

            telemetry.addData("Belt moving down: ", belt.getCurrentPosition());
        }      // Turn belt forward when A is pressed

        else if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            belt.setPower(0.001);
            belt2.setPower(0.001);

            telemetry.addData("Belt moving down: ", belt.getCurrentPosition());
        }


       if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            //clawOffset += CLAW_SPEED;
           claw.setPosition(0.22);

        } else if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            //clawOffset -= CLAW_SPEED;
           claw.setPosition(0.5);
        }

        //clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        //wrist.setPosition(0.5 + clawOffset);
        //claw.setPosition(0.5 - clawOffset);


        /*telemetry.addData("Claw offset: ",  "Offset = %.2f", clawOffset);
        telemetry.addData("Claw pos: ", claw.getPosition());
        telemetry.addData("Claw port: ", claw.getPortNumber());*/
        telemetry.addData("Wrist pos: ", wrist.getPosition());
        telemetry.addData("Wrist port: ", wrist.getPortNumber());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
