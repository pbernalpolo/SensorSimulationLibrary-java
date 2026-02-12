package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Vector2;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories2d.DifferentiableTrajectory2dOfPoint;



public class DifferentiableTrajectory3dOfPointFrom2d
    extends ContinuousTrajectory3dOfPointFrom2d
    implements DifferentiableTrajectory3dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    DifferentiableTrajectory2dOfPoint trajectory2d;  // the 2d trajectory
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public DifferentiableTrajectory3dOfPointFrom2d( DifferentiableTrajectory2dOfPoint theTrajectory2d )
    {
        super( theTrajectory2d );
        this.trajectory2d = theTrajectory2d;
    }



    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public Vector3 velocity()
    {
        Vector2 v = this.trajectory2d.velocity();
        return Vector3.fromComponents( v.x() , v.y() , 0.0 );
    }
    
    
    public Vector3 acceleration()
    {
        Vector2 a = this.trajectory2d.velocity();
        return Vector3.fromComponents( a.x() , a.y() , 0.0 );
    }
    
    
    public Vector3 jerk()
    {
        Vector2 j = this.trajectory2d.velocity();
        return Vector3.fromComponents( j.x() , j.y() , 0.0 );
    }
    
}
