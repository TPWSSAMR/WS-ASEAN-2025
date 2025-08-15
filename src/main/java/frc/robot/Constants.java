/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.studica.frc.Lidar;
import com.studica.frc.Lidar.Port;


public final class Constants 
{
    public enum ROBOTTYPE {
        FourWheel,
        TwoWheel,
        Mecanum,
        XDrive,
        SixWheel,
        KiwiDrive
    }

    public static final boolean ENABLEMOCKDS = true;
    public static final ROBOTTYPE CURRENTROBOT = ROBOTTYPE.KiwiDrive;

    //! Debug Section
    public static final boolean DEBUG_DRIVETRAIN = false;    // Enables the motor debug section
    public static final boolean DEBUG_LIDAR = false;    // Enables the lidar debug section
    public static final boolean DEBUG_CONTROLPANEL = false; //Enables the control panel debug sections

    public static final int TITAN_ID = 42;

    //! Mecanum + Six Wheel + XDrive
    // public static final int FRONT_LEFT_MOTOR        = 2;
    // public static final int FRONT_RIGHT_MOTOR       = 0;
    // public static final int BACK_LEFT_MOTOR         = 3;
    // public static final int BACK_RIGHT_MOTOR        = 1;

    // ! Four Wheel + Two Wheel
    // public static final int FRONT_LEFT_MOTOR        = 0;
    // public static final int FRONT_RIGHT_MOTOR       = 1;
    // public static final int BACK_LEFT_MOTOR         = 2;
    // public static final int BACK_RIGHT_MOTOR        = 3;


    //! Kiwi Drive
    // public static final int FRONT_LEFT_MOTOR        = 1; //! Left
    // public static final int FRONT_RIGHT_MOTOR       = 3; //! Right
    // public static final int BACK_LEFT_MOTOR         = 2; //! Back
    // public static final int BACK_RIGHT_MOTOR        = 0; //! Unused motor

    //! TEMPOOP
    public static final int FRONT_LEFT_MOTOR        = 1; //! Left
    public static final int FRONT_RIGHT_MOTOR       = 3; //! Right
    public static final int BACK_LEFT_MOTOR         = 2; //! Back
    public static final int BACK_RIGHT_MOTOR        = 0; //! Unused motor


    public static final class lidar
    {
       
    }

    public static final class controlPanel
    {


    }

    //! Gamepad Constants
    public static final class Gamepad
    {
        // USB PORT Constant
        public static final int USB_PORT = 0;

        // Studica Robotics Multi-Controller
        public static final int[] BUTTONS = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }; // A, B, X, Y, Left Bumper, Right Bumper, Back, Start, Left Analog Click, Right Analog Click
        public static final int[] JOYSTICKS = { 0, 1, 2, 3, 4, 5 }; // LeftX, LeftY, LeftTrigger, RightTrigger, RightX, RightY

        // Studica Robotics MODEL CUH-ZCT1U Wireless Controller
        // public static final int[] BUTTONS = { 2, 3, 1, 4, 5, 6, 9, 10, 11, 12 , 13, 14}; // A, B, X, Y, Left Bumper, Right Bumper, Back, Start, Left Analog Click, Right Analog Click, Off Button, Touchpad Click
        // public static final int[] JOYSTICKS = { 0, 1, 3, 4, 2, 5 }; // LeftX, LeftY, LeftTrigger, RightTrigger, RightX, RightY

        // Logitech Gamepad F310
        // public static final int[] BUTTONS = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 }; // A, B, X, Y, Left Bumper, Right Bumper, Back, Start, Left Analog Click, Right Analog Click
        // public static final int[] JOYSTICKS = { 0, 1, 2, 3, 4, 5 }; // LeftX, LeftY, LeftTrigger, RightTrigger, RightX, RightY
    }
}
