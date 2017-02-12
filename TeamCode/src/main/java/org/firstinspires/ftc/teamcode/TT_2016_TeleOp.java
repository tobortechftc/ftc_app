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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
@TeleOp(name = "TeleOp-2016", group = "TT-LN-OP")
public class TT_2016_TeleOp extends TT_2016_Hardware {
    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TELEOP);
        Boolean hanging_there = false;
        int test_count = 0;
        int delay_count = 0;
        int monitor_count = 0;
        double cur_SH_power = 0;

        speedScale = (float) 1.0; // control the initial chassis speed

        waitForStart();

        // StraightIn(0.4, 10);
        cdim.setDigitalChannelState(LED_CHANNEL, true);

        // right side beacon push
        set_right_beacon_side(RIGHT_BEACON_SIDE_PRESS);

        while (opModeIsActive()) {
            if (true) {
                ; // skip turn testing
            } else if (test_count == 0) {
                // StraightR(1.0, 1);
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 100) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 200) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 300) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 400) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 500) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 600) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 700) {
                TurnLeftD(1, 90, true);
                test_count++;
                continue;

            } else if (test_count == 1000) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1100) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1200) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1300) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1400) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1500) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1600) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else if (test_count == 1700) {
                TurnRightD(1, 90, true);
                test_count++;
                continue;
            } else {
                test_count++;
            }
            if (delay_count > 0)
                delay_count--;

            if (true) {
                monitor_count++;
                if (monitor_count % 10000 == 0) {
                    adjustShooterPower();
                }
                if (monitor_count % 100 == 0) {
                    double ul_val = ultra.getUltrasonicLevel();
                    shooting_range = (ul_val >= ULTRA_GOAL_MIN && ul_val <= ULTRA_GOAL_MAX);
                }
            }
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.

            // Use speedScale to control the speed
            rightPower = (float) ((float) scaleInput(right * speedScale));
            leftPower = (float) ((float) scaleInput(left * speedScale));

            // write the values to the motors
            motorR.setPower(rightPower);
            motorL.setPower(leftPower);
            sweeper.setPower(SW_power);
            shooter.setPower(cur_SH_power);

            if (gamepad1.b) { // sweeper backward
                if (delay_count > 0) { // no_action
                    ;
                } else if (SW_power > 0.1 || SW_power < -0.1) {
                    SW_power = (float) 0.0;
                } else {
                    SW_power = (float) 1.0;
                    set_left_beacon(LEFT_BEACON_INIT);
                    set_right_beacon(RIGHT_BEACON_INIT);
                }
                //cur_SH_power = (float) 0;
                delay_count = 200;
                sleep(5);
            } else if (gamepad1.x) { // sweeper forward
                if (delay_count > 0) { // no action
                    ;
                } else if (SW_power < -0.1) {
                    SW_power = (float) 0.0;
                    set_gate(GATE_CLOSED);
                } else {
                    SW_power = (float) -1.0;
                    set_gate(GATE_OPEN);
                    set_left_beacon(LEFT_BEACON_INIT);
                    set_right_beacon(RIGHT_BEACON_INIT);
                }
                //cur_SH_power = (float) 0;
                delay_count = 200;
                sleep(5);
            }

            // update the speed of the chassis, or stop tape slider
            if (gamepad1.a && gamepad1.y) { // back = fix speedscale 0.35
                speedScale = (float) 0.35;
                sleep(5);
            } else if (gamepad1.a) {
                // if the A button is pushed on gamepad1, decrease the speed
                // of the chassis
                if (speedScale > 0.2) {
                    speedScale -= 0.005;
                    sleep(5);
                }
            } else if (gamepad1.y) {
                // if the Y button is pushed on gamepad1, increase the speed
                // of the chassis
                if (speedScale < 1) {
                    speedScale += 0.005;
                    sleep(5);
                }
            }

            if (gamepad2.right_trigger > 0.1) { // *reconfigure later*
                set_right_beacon_side(RIGHT_BEACON_SIDE_PRESS);
                sleep(300);
            }
            if (gamepad2.right_bumper) { //
                set_right_beacon_side(RIGHT_BEACON_SIDE_DOWN);
            }

            if (gamepad2.left_trigger > 0.1) { // left climber down
                set_left_beacon_side(LEFT_BEACON_SIDE_PRESS);
                sleep(300);
            }
            if (gamepad2.left_bumper) { // left climber up
                set_left_beacon_side(LEFT_BEACON_SIDE_DOWN);

            }
            if (true) {
                // disable dpad
            } else if (gamepad1.dpad_down) { // both beacon pushers down
                set_left_beacon(LEFT_BEACON_INIT);
                set_right_beacon(RIGHT_BEACON_INIT);
                sweeper.setPower(-0.5);
                sleep(100);
                sweeper.setPower(0);
                sleep(400);
            } else if (gamepad1.dpad_up) { // both beacon pushers up
                set_left_beacon(LEFT_BEACON_PRESS);
                set_right_beacon(RIGHT_BEACON_PRESS);
                sweeper.setPower(0.5);
                sleep(100);
                sweeper.setPower(0);
                sleep(400);
            } else if (gamepad1.dpad_left) {
                if (Math.abs(left_beacon_sv_pos - LEFT_BEACON_INIT) < 0.1) {
                    set_left_beacon(LEFT_BEACON_PRESS);
                } else {
                    set_left_beacon(LEFT_BEACON_INIT);
                }
                sleep(500);
            } else if (gamepad1.dpad_right) {
                if (Math.abs(right_beacon_sv_pos - RIGHT_BEACON_INIT) < 0.1) {
                    set_right_beacon(RIGHT_BEACON_PRESS);
                } else {
                    set_right_beacon(RIGHT_BEACON_INIT);
                }
                sleep(500);
            }
            if (gamepad2.left_stick_y > 0.1) {
                linear_slider.setPower(1);
            } else if (gamepad2.left_stick_y < -0.1) {
                linear_slider.setPower(-1);
            } else {
                linear_slider.setPower(0);
            }
            if (true){
                if (gamepad2.right_stick_button){
                    TurnRightD(0.4, 90, true);
                }
                if (gamepad2.left_stick_button){
                    TurnLeftD(0.4, 90, true);
                }
            }

            if (gamepad2.x) { // shooter on
                adjustShooterPower();
                //SW_power = (float) 0;
                shooter.setPower(0.5);
                cur_SH_power = (float) SH_power;
                sleep(5);
            } else if (gamepad2.b) { // shooter off
                //SW_power = (float) 0;
                cur_SH_power = (float) 0;
                set_golf_gate(GOLF_GATE_CLOSED);
            }

            if (gamepad2.x || gamepad2.b) {

            }

            if (gamepad2.y) {
                set_slider_gate(0.5);
            }
            if (gamepad2.a) {
                set_slider_gate(0.05);
            }
            if (gamepad1.right_trigger > 0.1) {
                set_golf_gate(GOLF_GATE_OPEN);
                sleep(200);
                set_golf_gate(GOLF_GATE_CLOSED);
                //set_gate(GATE_OPEN);
                //sleep(500);
                //set_gate(GATE_CLOSED);
            }
            if (gamepad1.right_bumper) {
                if (golf_gate_sv_pos == GOLF_GATE_CLOSED) {
                    set_golf_gate(GOLF_GATE_OPEN);
                } else {
                    set_golf_gate(GOLF_GATE_CLOSED);
                }
                if (true) {
                } else if (gate_sv_pos == GATE_CLOSED) {
                    set_gate(GATE_OPEN);
                } else {
                    set_gate(GATE_CLOSED);
                }
                sleep(50);
            }
            if (gamepad1.left_bumper) {
                set_pusher(PUSHER_UP);
            }
            if (gamepad1.left_trigger > 0.1) {
                push_ball();
                //set_gate(GATE_OPEN);
                sleep(550);
                set_golf_gate(GOLF_GATE_OPEN);
                sleep(200);
                set_golf_gate(GOLF_GATE_CLOSED);
            }


            if (gamepad2.dpad_up) {
                set_left_beacon(LEFT_BEACON_PRESS);
                set_right_beacon(RIGHT_BEACON_PRESS);
                sweeper.setPower(0.5);
                sleep(100);
                sweeper.setPower(0);
                sleep(400);
            } else if (gamepad2.dpad_left) {
                if (Math.abs(left_beacon_sv_pos - LEFT_BEACON_INIT) < 0.1) {
                    set_left_beacon(LEFT_BEACON_PRESS);
                } else {
                    set_left_beacon(LEFT_BEACON_INIT);
                }
                sleep(500);
            } else if (gamepad2.dpad_right) {
                if (Math.abs(right_beacon_sv_pos - RIGHT_BEACON_INIT) < 0.1) {
                    set_right_beacon(RIGHT_BEACON_PRESS);
                } else {
                    set_right_beacon(RIGHT_BEACON_INIT);
                }
                sleep(500);
            } else if (gamepad2.dpad_down) {
                set_left_beacon(LEFT_BEACON_INIT);
                set_right_beacon(RIGHT_BEACON_INIT);
                sweeper.setPower(-0.5);
                sleep(100);
                sweeper.setPower(0);
                sleep(400);
            }
            if (detectWhite()) {
                detectwhite = 1;
            } else {
                detectwhite = 0;
            }

            show_telemetry();
        }
        cdim.setDigitalChannelState(LED_CHANNEL, false);
    }
}
