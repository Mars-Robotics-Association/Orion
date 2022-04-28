package org.firstinspires.ftc.teamcode.Navigation.FieldState;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.List;

public class FieldStateWrapper
{
    OpMode opMode;
    FileInputStream fieldStateIn;
    FileOutputStream fieldStateOut;

    List<FieldObject> objects;
    List<FieldObject> updatedObjects;

    double dynamicObjectDecaySeconds = 30;

    public void AddObjects(FieldObject[] objectsToAdd, OpMode setOpMode){
        opMode = setOpMode;
        for (FieldObject f : objectsToAdd) {
            updatedObjects.add(f);
        }
    }

    public void Update(){
        //decide what objects can be discarded
        for(FieldObject f : objects){
            if(f.objectType == FieldObject.ObjectType.PROBABILITY) objects.remove(f); //remove all probability objects

            if(f.objectType == FieldObject.ObjectType.DYNAMIC){
                if(opMode.getRuntime() >= f.timeUpdated + dynamicObjectDecaySeconds) objects.remove(f); //remove dynamic object if time is up
            }
        }

        //add or modify any remaining objects
        for(FieldObject uf : updatedObjects){
            boolean modified = false;
            for(FieldObject f : objects){ //update any previous objects with new values
                if(uf.id == f.id){
                    f = uf;
                    modified = true;
                }
            }
            if(!modified) objects.add(uf);//if object was not already on list, add it
        }
    }
}
