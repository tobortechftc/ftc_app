/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
@TeleOp(name = "Trike-2016", group = "TT-LN-OP")
public class Trike_TeleOp extends LinearOpMode {

    TrikeHardware robot = new TrikeHardware();
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        Boolean hanging_there = false;
        int test_count = 0;
        int delay_count = 0;
        int monitor_count = 0;
        double cur_SH_power = 0;
        boolean is_forward = true;
        double current_turn_angle = robot.TRIKE_TURNING_INIT;

        robot.speedScale = (float) 1.0; // control the initial chassis speed

        waitForStart();

        // StraightIn(0.4, 10);

        while (opModeIsActive()) {

            float drive = -gamepad1.left_stick_y;

            drive = Range.clip(drive, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // Use speedScale to control the speed
            robot.drivePower =(float)(drive * robot.speedScale);

            // write the values to the motors
            robot.trike_drive.setPower(robot.drivePower);

            if(gamepad1.left_bumper){
                robot.set_trike_turning(robot.TRIKE_TURNING_INIT);
            }

            if(gamepad1.right_bumper){
                robot.set_trike_turning(robot.TRIKE_TURNING_RIGHT);
            }

            if(gamepad1.a){
                if(is_forward){
                    robot.set_trike_turning(robot.TRIKE_TURNING_RIGHT);
                    is_forward = false;
                }
                else{
                    robot.set_trike_turning(robot.TRIKE_TURNING_INIT);
                    is_forward = true;
                }
                sleep(50);
            }

            if(gamepad1.right_trigger > 0.1){
                current_turn_angle += 0.01;
                robot.set_trike_turning(current_turn_angle);
            }

            if(gamepad1.left_trigger > 0.1){
                current_turn_angle -= 0.01;
                robot.set_trike_turning(current_turn_angle);
            }
        }
    }
}
