package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Quaternion;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;



public class CircleTrajectory
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double t;  // time at which the trajectory will be evaluated
    private double R;  // distance from center of torus to center of tube
    private double phiDot;  // derivative of phi angle (phi = phiDot * t)
    private double phiDot2;  // phiDot * phiDot
    private double phiDot3;  // phiDot * phiDot * phiDot
    private double sp;  // sin( phi )
    private double cp;  // cos( phi )
    private UnitQuaternion q;  // quaternion that represents the orientation as a rotation transformation from the reference frame to inertial (qIF)

    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public CircleTrajectory( double theR , double phiDerivative )
    {
        this.R = theR;
        this.phiDot = phiDerivative;
        this.phiDot2 = this.phiDot * this.phiDot;
        this.phiDot3 = this.phiDot2 * this.phiDot;
    }



    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////

    public void setTime( double time )
    {
        double phi = this.phiDot * time;
        // position
        this.sp = Math.sin( phi );
        this.cp = Math.cos( phi );
        // orientation
        double sp05 = Math.sin( phi/2.0 );
        double cp05 = Math.cos( phi/2.0 );
        Quaternion p = Quaternion.fromComponents( cp05 , 0.0 , 0.0 , sp05 );
        this.q = UnitQuaternion.fromNormalizedQuaternion( p );
        this.t = time;
    }
    
    
    public double time()
    {
        return this.t;
    }

    
    public Vector3 position()
    {
        return Vector3.fromComponents(
        		this.R * this.cp ,
                this.R * this.sp ,
                0.0 );
    }
    
    
    public Vector3 velocity()
    {
        return Vector3.fromComponents(
        		- this.R * this.sp * this.phiDot ,
                this.R * this.cp * this.phiDot ,
                0.0 );
    }


    public Vector3 acceleration()
    {
        return Vector3.fromComponents(
        		- this.R * this.cp * this.phiDot2  ,
                - this.R * this.sp * this.phiDot2 ,
                0.0 );
    }
    
    
    public Vector3 jerk()
    {
        return Vector3.fromComponents(
        		this.R * this.sp * this.phiDot3 ,
                - this.R * this.cp * this.phiDot3 ,
                0.0 );
    }

    
    public UnitQuaternion orientation()
    {
        return this.q;
    }

    
    public Vector3 angularVelocity()
    {
        return Vector3.fromComponents( 0.0 , 0.0 , this.phiDot );
    }


    public Vector3 angularAcceleration()
    {
        return Vector3.fromComponents( 0.0 , 0.0 , 0.0 );
    }
    
}
