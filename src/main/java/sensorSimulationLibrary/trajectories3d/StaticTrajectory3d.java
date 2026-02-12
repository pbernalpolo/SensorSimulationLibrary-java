package sensorSimulationLibrary.trajectories3d;

import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;

public class StaticTrajectory3d
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double t;  // time at which the trajectory will be evaluated
    private UnitQuaternion q;  // orientation of reference frame
    private Vector3 r;  // position of the reference frame
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public StaticTrajectory3d()
    {
        this.r = Vector3.zero();
    }



    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setOrientation( UnitQuaternion theUnitQuaternion )
    {
        this.q = theUnitQuaternion;
    }
    
    
    public void setPosition( Vector3 thePosition )
    {
        this.r = thePosition;
    }
    
    
    public void setTime( double time )
    {
        this.t = time;
    }
    
    
    public double time()
    {
        return this.t;
    }
    
    
    public Vector3 position()
    {
        return this.r;
    }
    
    
    public Vector3 velocity()
    {
        return Vector3.zero();
    }


    public Vector3 acceleration()
    {
        return Vector3.zero();
    }
    
    
    public Vector3 jerk()
    {
        return Vector3.zero();
    }
    
    
    public UnitQuaternion orientation()
    {
        return this.q;
    }
    
    
    public Vector3 angularVelocity()
    {
        return Vector3.zero();
    }
    
}
