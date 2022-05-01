package org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Attachments;

import org.firstinspires.ftc.teamcode.Core.MechanicalControlToolkit.Basic.DCMotorArray;

public interface EncoderActuatorProfile
{
    DCMotorArray motors(); //Motors to control
    double maxRots(); //The maximum number of end rotations taking the gear ratio into account.
    double minRots(); //Same as above but the minimum. Usually zero.
    double gearRatio(); //The ratio to apply for the gear ratio excluding the motor gearbox. If your thing is geared with a 30 tooth gear to and 90 tooth one, this would be 3.
    double encoderResolution(); //The resolution of the encoder. Go to the encoder's store page. 537.7 is gobuilda 19.2:1s.
    boolean reverseEncoder(); //Whether to reverse the data from the encoder.
    boolean useEncoder(); //whether to use an encoder at all.
}
