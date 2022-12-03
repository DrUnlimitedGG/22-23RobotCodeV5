

package org.firstinspires.ftc.teamcode.drive.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name="ClawTesting", group="Testing")
public class ClawTesting extends OpMode
{
    private Servo claw = null;
    private Servo wrist = null;

    double position = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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

        claw.setPosition(0.15);
        wrist.setPosition(1);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad2.right_bumper && !gamepad2.left_bumper) {
            //clawOffset += CLAW_SPEED;
            claw.setPosition(0.225);

            /*telemetry.addData("Status: ", "Opening");
            telemetry.addData("Position: ", claw.getPosition());*/


        }

        if (gamepad2.left_bumper && !gamepad2.right_bumper) {
            //clawOffset -= CLAW_SPEED;
            claw.setPosition(0.15);

            /*telemetry.addData("Status: ", "Closing");
            telemetry.addData("Position: ", claw.getPosition());*/


        }

        if (gamepad2.y && !gamepad2.x) {
            if (position + 0.005 < 0.2) {
                position = position + 0.001;
                wrist.setPosition(position);

                telemetry.addData("We raising that sh", " True");
            }

        }

        if (gamepad2.x && !gamepad2.y) {
            if (position - 0.005 > 0) {
                position = position - 0.005;
                wrist.setPosition(position);

                telemetry.addData("We lowering that sh", " True");

            }

        }

        telemetry.addData("Position: ", position);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        claw.setPosition(0.1);
    }

}