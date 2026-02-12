package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Quaternion;
import numericalLibrary.types.Vector3;



public class CylinderTrajectory
    extends TrajectoryOfReferenceFrameFromOrientation
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double t;  // time at which the trajectory will be evaluated
    private double R;  // radius of the cylinder
    private double h;  // height of the cylinder
    private double thetaDot;  // derivative of theta angle (theta = thetaDot * t)
    private double phiDot;  // derivative of phi angle (phi = phiDot * t)
    private double thetaDot2;  // thetaDot * thetaDot
    private double phiDot2;  // phiDot * phiDot
    private double thetaDot3;  // thetaDot * thetaDot * thetaDot
    private double phiDot3;  // phiDot * phiDot * phiDot
    private double st;  // sin( theta )
    private double ct;  // cos( theta )
    private double sp;  // sin( phi )
    private double cp;  // cos( phi )

    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public CylinderTrajectory( double theR , double the_h , double thetaDerivative , double phiDerivative )
    {
        this.R = theR;
        this.h = the_h;
        this.thetaDot = thetaDerivative;
        this.phiDot = phiDerivative;
        this.thetaDot2 = this.thetaDot * this.thetaDot;
        this.phiDot2 = this.phiDot * this.phiDot;
        this.thetaDot3 = this.thetaDot2 * this.thetaDot;
        this.phiDot3 = this.phiDot2 * this.phiDot;
    }



    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////

    public void setTime( double time )
    {
        double theta = this.thetaDot * time;
        double phi = this.phiDot * time;
        // position
        this.st = Math.sin( theta );
        this.ct = Math.cos( theta );
        this.sp = Math.sin( phi );
        this.cp = Math.cos( phi );
        // orientation
        double sp05 = Math.sin( phi/2.0 );
        double cp05 = Math.cos( phi/2.0 );
        this.q = Quaternion.fromComponents( cp05 , 0.0 , 0.0 , sp05 );
        this.qd = Quaternion.fromComponents( -sp05 , 0.0 , 0.0 , cp05 ).scaleInplace( this.phiDot/2.0 );
        this.qdd = Quaternion.fromComponents( -cp05 , 0.0 , 0.0 , -sp05 ).scaleInplace( this.phiDot2/4.0 );
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
                this.h * this.st );
    }
    
    
    public Vector3 velocity()
    {
        return Vector3.fromComponents(
        		-this.R * this.sp * this.phiDot ,
                this.R * this.cp * this.phiDot ,
                this.h * this.ct * this.thetaDot );
    }
    
    
    public Vector3 acceleration()
    {
        double ax  =  - this.R * this.cp * this.phiDot2;
        double ay  =  - this.R * this.sp * this.phiDot2;
        double az  =  - this.h * this.st * this.thetaDot2;
        return Vector3.fromComponents( ax , ay , az );
    }
    
    
    public Vector3 jerk()
    {
        double jx  =  this.R * this.sp * this.phiDot3;
        double jy  =  - this.R * this.cp * this.phiDot3;
        double jz  =  - this.h * this.ct * this.thetaDot3;
        return Vector3.fromComponents( jx , jy , jz );
    }
    
}
