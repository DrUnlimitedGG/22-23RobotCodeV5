

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name="ArmTesting", group="Testing")
public class ArmTesting extends OpMode
{
    private DcMotorEx belt = null; // Belt motor
    private DcMotorEx belt2 = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        belt2 = hardwareMap.get(DcMotorEx.class, "belt2");

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

        //claw.setPosition(0.22);
        //wrist.setPosition(0.5);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad2.dpad_right) {
            belt.setPower(0.5);
            belt2.setPower(0.5);
        }

        if (gamepad2.dpad_left) {
            belt.setPower(-0.5);
            belt2.setPower(-0.5);

        }


        telemetry.addData("Belt 1 Power: ", belt.getPower());
        telemetry.addData("Belt 1 Velocity: ", belt.getVelocity());
        telemetry.addData("Belt 1 Status: ", belt.getConnectionInfo());

        telemetry.addData("Belt 2 Power: ", belt2.getPower());
        telemetry.addData("Belt 2 Velocity: ", belt2.getVelocity());
        telemetry.addData("Belt 2 Status: ", belt2.getConnectionInfo());

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}
