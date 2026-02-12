package sensorSimulationLibrary.trajectories2d;


import numericalLibrary.types.Vector2;



// https://mathcurve.com/courbes2d.gb/cercle/cercle.shtml
public class CircleCurve
    implements DifferentiableTrajectory2dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double t;  // time at which the trajectory will be evaluated
    private double r;  // radius of the circle
    private double st;  // sin( t )
    private double ct;  // cos( t )
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public CircleCurve( double radiousParameter )
    {
        this.r = radiousParameter;
    }



    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////

    public void setTime( double time )
    {
        this.st = Math.sin( time );
        this.ct = Math.cos( time );
        this.t = time;
    }
    
    
    public double time()
    {
        return this.t;
    }

    
    public Vector2 position()
    {
        return Vector2.fromComponents(
                this.ct ,
                this.st ).scaleInplace( this.r );
    }
    
    
    public Vector2 velocity()
    {
        return Vector2.fromComponents(
                - this.st ,
                this.ct ).scaleInplace( this.r );
    }


    public Vector2 acceleration()
    {
        return Vector2.fromComponents(
                - this.ct ,
                - this.st ).scaleInplace( this.r );
    }
    
    
    public Vector2 jerk()
    {
        return Vector2.fromComponents(
                this.st ,
                - this.ct ).scaleInplace( this.r );
    }
    
}
