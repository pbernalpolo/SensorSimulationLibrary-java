package sensorSimulationLibrary.sensors.camera;

import java.util.ArrayList;
import java.util.List;

import numericalLibrary.types.MatrixReal;
import numericalLibrary.types.UnitQuaternion;
import numericalLibrary.types.Vector2;
import numericalLibrary.types.Vector3;
import sensorSimulationLibrary.trajectories3d.ContinuousTrajectory3dOfReferenceFrame;



public class PlaneImagePointsCamera
{
	////////////////////////////////////////////////////////////////
	/// PRIVATE VARIABLES
	////////////////////////////////////////////////////////////////
	
    /**
     * {@link UnitQuaternion} that defines the rotation transformation from sensor to trajectory frame.
     */
    private UnitQuaternion qTS;
    
    /**
     * {@link Vector3} that defines the position of the sensor relative to the trajectory frame.
     */
    private Vector3 rTTS;
    
    private Vector3 planeNormal;
    
    private double planeNormalDotPlanePoint;
    
    private List<Vector2> imagePointList;
    
	private double timeIncrement;
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PUBLIC CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    
    public PlaneImagePointsCamera()
    {
        this.qTS = UnitQuaternion.one();
        this.rTTS = Vector3.zero();
        this.planeNormal = Vector3.zero();
        this.planeNormalDotPlanePoint = 0.0;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PUBLIC METHODS
    ////////////////////////////////////////////////////////////////
    
    public void setTimeIncrement( double timeIncrement )
    {
    	this.timeIncrement = timeIncrement;
    }
    
    
    public void setPlaneNormalAndPoint( Vector3 planeNormal , Vector3 planePoint )
    {
    	this.planeNormal.setTo( planeNormal );
    	this.planeNormalDotPlanePoint = planeNormal.dot( planePoint );
    }
    
    
    public void setImagePoints( List<Vector2> imagePointList )
    {
    	this.imagePointList = imagePointList;
    }
    
    
    public void setSensorOrientationToTrajectoryFrame( UnitQuaternion sensorOrientationToTrajectoryFrame )
    {
        this.qTS.setTo( sensorOrientationToTrajectoryFrame );
    }
    
    
    public void setSensorPositionFromTrajectoryFrame( Vector3 sensorPositionFromTrajectoryFrame )
    {
        this.rTTS.setTo( sensorPositionFromTrajectoryFrame );
    }
    
    
    public List<Vector2> measure( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
        // Get plane points.
        List<Vector3> rOOP_list = this.getPlanePoints( trf , time );
        // Compute image points.
        trf.setTime( time );
        UnitQuaternion qOT = trf.orientation();
        Vector3 rOOT = trf.position();
        List<Vector2> output = new ArrayList<Vector2>();
        for( Vector3 rOOP : rOOP_list ) {
        	Vector3 rSSP = this.qTS.rotateWithInverse( qOT.rotateWithInverse( rOOP.subtract( rOOT ) ).subtractInplace( this.rTTS ) );
        	if( rSSP.z() <= 0.0 ) {
        		output.add( null );
        		continue;
        	}
        	Vector2 uv = Vector2.fromComponents( rSSP.x() , rSSP.y() ).scaleInplace( 1.0/rSSP.z() );
        	output.add( uv );
        }
        return output;
    }
    
    
    
    ////////////////////////////////////////////////////////////////
    /// PRIVATE METHODS
    ////////////////////////////////////////////////////////////////
    
    private List<Vector3> getPlanePoints( ContinuousTrajectory3dOfReferenceFrame trf , double time )
    {
    	trf.setTime( time - this.timeIncrement );
    	UnitQuaternion qOT = trf.orientation();
    	Vector3 rOOT = trf.position();
    	MatrixReal nT_R_ON = this.planeNormal.toMatrixAsRow().multiply( qOT.toRotationMatrix() );
    	
    	double rSiSiP_z_numerator = this.planeNormalDotPlanePoint - nT_R_ON.multiply( this.rTTS.toMatrixAsColumn() ).entry(0,0) - this.planeNormal.dot( rOOT );
    	List<Vector3> output = new ArrayList<Vector3>();
    	for( Vector2 uv : this.imagePointList ) {
    		Vector3 uv1 = Vector3.fromComponents( uv.x() , uv.y() , 1.0 );
    		double rSiSiP_z_denominator = nT_R_ON.multiply( this.qTS.rotate( uv1 ).toMatrixAsColumn() ).entry(0,0);
    		double rSiSiP_z = rSiSiP_z_numerator / rSiSiP_z_denominator;
    		Vector3 rSiSiP = uv1.scale( rSiSiP_z );
    		Vector3 rOOP = rOOT.add( qOT.rotate( this.rTTS.add( this.qTS.rotate( rSiSiP ) ) ) );
    		output.add( rOOP );
    	}
    	return output;
    }
    
}
