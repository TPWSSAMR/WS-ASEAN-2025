package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gamepad extends SubsystemBase {

    private Joystick controller;

    public Gamepad() {
        controller = new Joystick(Constants.Gamepad.USB_PORT);
    }

    public double getJoystickLeftX() {
        double joy = controller.getRawAxis(Constants.Gamepad.JOYSTICKS[0]);
        return (Math.abs(joy) < 0.05) ? 0 : joy;
    }

    public double getJoystickLeftY() {
        double joy = controller.getRawAxis(Constants.Gamepad.JOYSTICKS[1]);
        return (Math.abs(joy) < 0.05) ? 0 : joy;
    }

    public double getTriggerLeft() {
        double joy = controller.getRawAxis(Constants.Gamepad.JOYSTICKS[2]);
        return (Math.abs(joy) < 0.05) ? 0 : joy;
    }

    public double getTriggerRight() {
        double joy = controller.getRawAxis(Constants.Gamepad.JOYSTICKS[3]);
        return (Math.abs(joy) < 0.05) ? 0 : joy;
    }

    public double getJoystickRightX() {
        double joy = controller.getRawAxis(Constants.Gamepad.JOYSTICKS[4]);
        return (Math.abs(joy) < 0.05) ? 0 : joy;
    }

    public double getJoystickRightY() {
        double joy = controller.getRawAxis(Constants.Gamepad.JOYSTICKS[5]);
        return (Math.abs(joy) < 0.05) ? 0 : joy;
    }

    public boolean getButtonA() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[0]);
    }

    public boolean getButtonB() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[1]);
    }

    public boolean getButtonX() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[2]);
    }

    public boolean getButtonY() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[3]);
    }

    public boolean getBumperLeft() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[4]);
    }

    public boolean getBumperRight() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[5]);
    }

    public boolean getButtonBack() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[6]);
    }

    public boolean getButtonStart() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[7]);
    }

    public boolean getAnalogButtonLeft() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[8]);
    }

    public boolean getAnalogButtonRight() {
        return controller.getRawButton(Constants.Gamepad.BUTTONS[9]);
    }

    public int getPOV() {
        return controller.getPOV();
    }
}
