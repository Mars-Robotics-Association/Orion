package org.firstinspires.ftc.teamcode.Orion.FieldState;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.List;

class FieldStateWrapper
{
    FileInputStream fieldStateIn;
    FileOutputStream fieldStateOut;

    List<FieldObject> objects;
    List<FieldObject> updatedObjects;

    public void AddObjects(FieldObject[] objectsToAdd){
        for (FieldObject f : objectsToAdd) {
            updatedObjects.add(f);
        }
    }

    public void Update(){
        //decide what objects can be discarded
        //decide which objects need to be modified and modify them
        //add any remaining objects
    }
}
