package org.firstinspires.ftc.teamcode.Core.Input;


public interface ControllerInputListener {

    void APressed(double controllerNumber);
    void BPressed(double controllerNumber);
    void XPressed(double controllerNumber);
    void YPressed(double controllerNumber);

    void AHeld(double controllerNumber);
    void BHeld(double controllerNumber);
    void XHeld(double controllerNumber);
    void YHeld(double controllerNumber);

    void AReleased(double controllerNumber);
    void BReleased(double controllerNumber);
    void XReleased(double controllerNumber);
    void YReleased(double controllerNumber);

    void LBPressed(double controllerNumber);
    void RBPressed(double controllerNumber);
    void LTPressed(double controllerNumber);
    void RTPressed(double controllerNumber);

    void LBHeld(double controllerNumber);
    void RBHeld(double controllerNumber);
    void LTHeld(double controllerNumber);
    void RTHeld(double controllerNumber);

    void LBReleased(double controllerNumber);
    void RBReleased(double controllerNumber);
    void LTReleased(double controllerNumber);
    void RTReleased(double controllerNumber);
    
    void DUpPressed(double controllerNumber);
    void DDownPressed(double controllerNumber);
    void DLeftPressed(double controllerNumber);
    void DRightPressed(double controllerNumber);

    void DUpHeld(double controllerNumber);
    void DDownHeld(double controllerNumber);
    void DLeftHeld(double controllerNumber);
    void DRightHeld(double controllerNumber);

    void DUpReleased(double controllerNumber);
    void DDownReleased(double controllerNumber);
    void DLeftReleased(double controllerNumber);
    void DRightReleased(double controllerNumber);

    void LJSPressed(double controllerNumber);
    void RJSPressed(double controllerNumber);

    void LJSHeld(double controllerNumber);
    void RJSHeld(double controllerNumber);

    void LJSReleased(double controllerNumber);
    void RJSReleased(double controllerNumber);
}
