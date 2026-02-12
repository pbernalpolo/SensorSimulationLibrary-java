package sensorSimulationLibrary.trajectories2d;


import numericalLibrary.types.Vector2;



// https://mathcurve.com/courbes2d.gb/gerono/gerono.shtml
public class EightCurve
    implements DifferentiableTrajectory2dOfPoint
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double t;  // time at which the trajectory will be evaluated
    private double a;  // radius of the sphere
    private double st;  // sin( t )
    private double ct;  // cos( t )
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public EightCurve( double aParameter )
    {
        this.a = aParameter;
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
                this.st ,
                this.st * this.ct ).scaleInplace( this.a );
    }
    
    
    public Vector2 velocity()
    {
        return Vector2.fromComponents(
                this.ct ,
                this.ct * this.ct - this.st * this.st ).scaleInplace( this.a );
    }


    public Vector2 acceleration()
    {
        return Vector2.fromComponents(
                - this.st ,
                - 4.0 * this.ct * this.st ).scaleInplace( this.a );
    }
    
    
    public Vector2 jerk()
    {
        return Vector2.fromComponents(
                - this.ct ,
                - 4.0 * ( this.ct * this.ct - this.st * this.st ) ).scaleInplace( this.a );
    }
    
}
