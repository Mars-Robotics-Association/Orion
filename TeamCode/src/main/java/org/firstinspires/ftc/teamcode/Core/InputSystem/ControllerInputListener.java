package org.firstinspires.ftc.teamcode.Core.InputSystem;

public interface ControllerInputListener
{
    void ButtonPressed(int id, ControllerInput.Button button);
    void ButtonHeld(int id, ControllerInput.Button button);
    void ButtonReleased(int id, ControllerInput.Button button);
}
