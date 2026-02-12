package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Quaternion;
import numericalLibrary.types.Vector3;



public class TorusTrajectory
    extends TrajectoryOfReferenceFrameFromOrientation
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private double t;  // time at which the trajectory will be evaluated
    private double R;  // distance from center of torus to center of tube
    private double r;  // radius of the tube
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
    private double Rrctheta;  // R + r * cos(theta)

    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public TorusTrajectory( double theR , double the_r , double thetaDerivative , double phiDerivative )
    {
        this.R = theR;
        this.r = the_r;
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
        this.Rrctheta = this.R + this.r * this.ct;
        // orientation
        double st05 = Math.sin( theta/2.0 );
        double ct05 = Math.cos( theta/2.0 );
        double sp05 = Math.sin( phi/2.0 );
        double cp05 = Math.cos( phi/2.0 );
        Quaternion qphi = Quaternion.fromComponents( cp05 , 0 , 0 , sp05 );
        Quaternion qphiDot = Quaternion.fromComponents( -sp05 , 0 , 0 , cp05 ).scaleInplace( this.phiDot/2.0 );
        Quaternion qphiDotdot = Quaternion.fromComponents( -cp05 , 0.0 , 0.0 , -sp05 ).scaleInplace( this.phiDot2/4.0 );
        Quaternion qtheta = Quaternion.fromComponents( ct05 , 0 , -st05 , 0 );
        Quaternion qthetaDot = Quaternion.fromComponents( -st05 , 0 , -ct05 , 0 ).scaleInplace( this.thetaDot/2.0 );
        Quaternion qthetaDotdot = Quaternion.fromComponents( -ct05 , 0 , st05 , 0 ).scaleInplace( this.thetaDot2/4.0 );
        Quaternion q0 = Quaternion.fromComponents( 0.0 , 1.0/Math.sqrt(2.0) , 0.0 , 1.0/Math.sqrt(2.0) );
        this.q = qphi.multiply( qtheta ).multiply( q0 );
        this.qd = qphiDot.multiply( qtheta ).add( qphi.multiply( qthetaDot ) ).multiply( q0 );
        this.qdd = qphiDotdot.multiply( qtheta ).add( qphiDot.multiply( qthetaDot ).scaleInplace( 2.0 ) ).add( qphi.multiply( qthetaDotdot ) ).multiply( q0 );
        this.t = time;
    }
    
    
    public double time()
    {
        return this.t;
    }

    
    public Vector3 position()
    {
    	return Vector3.fromComponents(
    			this.Rrctheta * this.cp ,
                this.Rrctheta * this.sp ,
                this.r * this.st );
    }
    
    
    public Vector3 velocity()
    {
    	return Vector3.fromComponents(
    			- this.r * this.st * this.thetaDot * this.cp - this.Rrctheta * this.sp * this.phiDot ,
                - this.r * this.st * this.thetaDot * this.sp + this.Rrctheta * this.cp * this.phiDot ,
                this.r * this.ct * this.thetaDot );
    }


    public Vector3 acceleration()
    {
        double ax  =  - this.r * this.ct * this.thetaDot2 * this.cp +
                      + 2.0 * this.r * this.st * this.thetaDot * this.sp * this.phiDot +
                      - this.Rrctheta * this.cp * this.phiDot2 ;

        double ay  =  - this.Rrctheta * this.sp * this.phiDot2 +
                      - 2.0 * this.r * this.st * this.thetaDot * this.cp * this.phiDot +
                      - this.r * this.ct * this.thetaDot2 * this.sp ;

        double az  =  - this.r * this.st * this.thetaDot2;
        
        return Vector3.fromComponents( ax , ay , az );
    }
    
    
    public Vector3 jerk()
    {
        double jx  =  this.r * this.st * this.thetaDot3 * this.cp +
                      + 3.0 * this.r * this.ct * this.thetaDot2 * this.sp * this.phiDot +
                      + 3.0 * this.r * this.st * this.thetaDot * this.cp * this.phiDot2 +
                      + this.Rrctheta * this.sp * this.phiDot3 ;
        
        double jy  =  - this.Rrctheta * this.cp * this.phiDot3 +
                      - 3.0 * this.r * this.ct * this.thetaDot2 * this.cp * this.phiDot +
                      + 3.0 * this.r * this.st * this.thetaDot * this.sp * this.phiDot2 +
                      + this.r * this.st * this.thetaDot3 * this.sp ;
        
        double jz  =  - this.r * this.ct * this.thetaDot3;
        
        return Vector3.fromComponents( jx , jy , jz );
    }

}
