package org.firstinspires.ftc.teamcode.TestingOpModes;

public class OtherTest {

    public static void main(String args[])
    {

        Class c1 = new Class("This is a Class defined as Class");
        Class c2 = new Class2("This is a Class defined as Class2");
        Class2 c3 = new Class2("This is a Class2 defined as a CLass2");

        int number1 = c1.doThing(3);
        int number2 = c2.doThing(7);
        int number3 = ((Class2)c2).doOtherThing(2);
        int number4 = c3.doThing(4);
        int number5 = c3.doOtherThing(4);


    }
}
