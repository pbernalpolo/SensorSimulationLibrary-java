package sensorSimulationLibrary.util;


import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;



public class Pose
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    
    private UnitQuaternion qOF;
    
    private Vector3 rOOF;
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public Pose( UnitQuaternion orientationFromReferenceFrame , Vector3 positionFromReferenceFrame )
    {
        this.qOF = orientationFromReferenceFrame;
        this.rOOF = positionFromReferenceFrame;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public UnitQuaternion orientationFromReferenceFrame()
    {
        return this.qOF;
    }
    
    
    public Vector3 positionFromReferenceFrame()
    {
        return this.rOOF;
    }
    
}
