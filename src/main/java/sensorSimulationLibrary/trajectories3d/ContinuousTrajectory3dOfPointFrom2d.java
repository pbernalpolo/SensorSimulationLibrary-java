package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector2;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories2d.ContinuousTrajectory2dOfPoint;



public class ContinuousTrajectory3dOfPointFrom2d
    implements ContinuousTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    ContinuousTrajectory2dOfPoint trajectory2d;  // the 2d trajectory
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public ContinuousTrajectory3dOfPointFrom2d( ContinuousTrajectory2dOfPoint theTrajectory2d )
    {
        this.trajectory2d = theTrajectory2d;
    }



    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////

    public void setTime( double time )
    {
        this.trajectory2d.setTime( time );
    }
    
    
    public double time()
    {
        return this.trajectory2d.time();
    }

    
    public Vector3 position()
    {
        Vector2 r = this.trajectory2d.position();
        return Vector3.fromComponents( r.x() , r.y() , 0.0 );
    }
    
}
