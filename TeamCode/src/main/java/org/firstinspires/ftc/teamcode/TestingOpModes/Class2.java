package org.firstinspires.ftc.teamcode.TestingOpModes;

public class Class2 extends Class{

    public Class2(String w) {
        super(w);
    }

    public int doOtherThing(int number)
    {
        number = super.doThing(number);
        number = number+3;
        return number;
    }
}
