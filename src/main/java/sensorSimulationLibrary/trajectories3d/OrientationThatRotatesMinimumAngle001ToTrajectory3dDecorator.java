package sensorSimulationLibrary.trajectories3d;


import numericalLibrary.types.Quaternion;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector3;



public class OrientationThatRotatesMinimumAngle001ToTrajectory3dDecorator
    implements DifferentiableTrajectory3dOfReferenceFrame
{
    ////////////////////////////////////////////////////////////////
    // PRIVATE VARIABLES
    ////////////////////////////////////////////////////////////////
    private DifferentiableTrajectory3dOfPoint wrappedTrajectory;  // the DifferentiableTrajectory2dOfPoint wrapped by this class
    private DifferentiableTrajectory3dOfPoint trajectoryToGenerateOrientation;  // the DifferentiableTrajectory3dOfPoint used to generate the orientation for the reference frame
    private Vector3 v;  // vector used to generate orientation. Stored to avoid recomputation.
    private UnitQuaternion uq;  // quaternion that represents orientation. Stored to avoid recomputation.
    private boolean dirtyFlag;  // true if we need to recompute
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////

    public OrientationThatRotatesMinimumAngle001ToTrajectory3dDecorator(
            DifferentiableTrajectory3dOfPoint wrappedDifferentiableTrajectory3dOfPoint ,
            DifferentiableTrajectory3dOfPoint trajectory3dUsedToGenerateOrientation )
    {
        this.wrappedTrajectory = wrappedDifferentiableTrajectory3dOfPoint;
        this.trajectoryToGenerateOrientation = trajectory3dUsedToGenerateOrientation;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTime( double theTime )
    {
        this.wrappedTrajectory.setTime( theTime );
        this.trajectoryToGenerateOrientation.setTime( theTime );
        this.dirtyFlag = true;
    }
    
    
    public double time()
    {
        return this.wrappedTrajectory.time();
    }
    
    
    public Vector3 position()
    {
        return this.wrappedTrajectory.position();
    }
    
    
    public Vector3 velocity()
    {
        return this.wrappedTrajectory.velocity();
    }
    
    
    public Vector3 acceleration()
    {
        return this.wrappedTrajectory.acceleration();
    }
    
    
    public Vector3 jerk()
    {
        return this.wrappedTrajectory.jerk();
    }
    
    
    public UnitQuaternion orientation()
    {
        this.clean();
        return this.uq;
    }
    
    
    public Vector3 angularVelocity()
    {
        /* Compute unit quaternion time derivative:
         * q = q( p( v ) )  =>  dq_i/dt = dq_i/dp_j dp_j/dv_k dv_k/dt
         * with
         * q(p) = p/||p||
         * p(v) = ( v_z + ||v|| )
         *        ( -v_y        )
         *        ( v_x         )
         * so
         * dp_j/dv_k = ( v_x/||v||  v_y/||v||   1 + v_z/||v|| )
         *             ( 0          -1          0             )
         *             ( 1          0           0             )
         *             ( 0          0           0             )
         * dp_j/dt = dp_j/dv_k dv_k/dt = ( dv_z/dt + v · dv/dt / ||v|| )
         *                               ( -dv_y/dt                    )
         *                               ( dv_x/dt                     )
         *                               ( 0                           )
         * dq_i/dp_j = ( I - q q^T )/||p||
         */
        // First, compute dp_j/dt
        Vector3 vDot = this.trajectoryToGenerateOrientation.velocity();
        double vnorm = this.v.norm();
        if( !( vnorm > 0.0 ) ) {
            throw new IllegalArgumentException( "[OrientationThatRotatesMinimumAngle001ToTrajectory3dDecorator.angularVelocity] Orientation cannot be generated from trajectory that contains (0,0,0)." );
        }
        Quaternion pDot = Quaternion.fromScalarAndVectorPart(
                    vDot.z() + this.v.scale( 1.0/vnorm ).dot( vDot ) ,
                    Vector3.fromComponents( - vDot.y() , vDot.x() , 0.0 ) );
        // Then, compute dq/dt
        double pnorm = Math.sqrt( 2.0 * vnorm * ( vnorm + this.v.z() ) );
        Quaternion q = this.uq.quaternion();
        Quaternion qDot = pDot.subtract( q.scale( q.dot( pDot ) ) ).scale( 1.0/pnorm );
        // Finally, compute omega using q and qDot.
        return q.conjugate().multiply( qDot ).vectorPart().scaleInplace( 2.0 );
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    // PRIVATE METHODS
    ////////////////////////////////////////////////////////////////
    
    private void clean()
    {
        if( this.dirtyFlag ) {
            // Get position used to generate orientation.
            this.v = this.trajectoryToGenerateOrientation.position();
            // Compute unit quaternion that represents orientation.
            this.uq = UnitQuaternion.thatRotatesMinimumAngle001To( this.v );
            // Now we are clean.
            this.dirtyFlag = false;
        }
    }

}
