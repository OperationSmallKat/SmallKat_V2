import java.time.Duration;
import java.util.ArrayList;

import javafx.application.Platform;

import org.reactfx.util.FxTimer;

import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics;
import com.neuronrobotics.sdk.addons.kinematics.MobileBase;
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR;
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR;
import com.neuronrobotics.sdk.util.ThreadUtil;
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine;
import com.neuronrobotics.sdk.common.Log;
import Jama.Matrix;

enum WalkingState {
    Rising,ToHome,ToNewTarget,Falling
}

if(args==null){
	double stepOverHeight=5;
	long stepOverTime=400;
	Double zLock=-12;
	Closure calcHome = { DHParameterKinematics leg -> 
			TransformNR h=leg.calcHome() 
	 		TransformNR  legRoot= leg.getRobotToFiducialTransform()
			TransformNR tr = leg.forwardOffset(new TransformNR())
			tr.setZ(zLock)
			//Bambi-on-ice the legs a bit
			if(legRoot.getY()>0){
				//tr.translateY(5)
			}else{
				//tr.translateY(-5)
			}
			
			return tr;
	
	}
	boolean usePhysicsToMove = true;
	long stepCycleTime =500
	
	args =  [stepOverHeight,stepOverTime,zLock,calcHome,usePhysicsToMove,stepCycleTime]
}

return new com.neuronrobotics.sdk.addons.kinematics.IDriveEngine (){
	boolean resetting=false;
	double stepOverHeight=(double)args.get(0);
	long stepOverTime=(long)args.get(1);
	private Double zLock=(Double)args.get(2);
	Closure calcHome =(Closure)args.get(3);
	boolean usePhysics=(args.size()>4?((boolean)args.get(4)):false);
	long stepCycleTime=args.get(5)
	long timeOfCycleStart= System.currentTimeMillis();
	int stepCycyleActiveIndex =0
	int numStepCycleGroups = 2
	int numlegs;
	TransformNR [] home=null;
	ArrayList<DHParameterKinematics> legs;
	HashMap<Integer,ArrayList<DHParameterKinematics> > cycleGroups=new HashMap<>();
	TransformNR previousGLobalState;
	TransformNR target;
	RotationNR rot;
	int resettingindex=0;
	private long reset = System.currentTimeMillis();
	
	Thread stepResetter=null;
	boolean threadDone=false
	WalkingState state= WalkingState.Rising
	MobileBase source 
	TransformNR newPose
	long miliseconds
	boolean timout = true
	public void resetStepTimer(){
		reset = System.currentTimeMillis();
	}
	
	public void walkingCycle(){
		if(reset+miliseconds< System.currentTimeMillis()){
			timout = true
		}else
			timout = false
		if((timeOfCycleStart+stepCycleTime)>System.currentTimeMillis()){
			timeOfCycleStart=System.currentTimeMillis()
			stepCycyleActiveIndex++
			if(stepCycyleActiveIndex==numStepCycleGroups){
				stepCycyleActiveIndex=0;
			}
			//println "Cycle = "+cycleGroups.get(stepCycyleActiveIndex)
		}else{
			//println " Waiting till "+(timeOfCycleStart+stepCycleTime)+" is "+System.currentTimeMillis()leg
		}
		double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
		double gaitPercentage = timeRemaining/(double)(stepCycleTime)
		double gaitIntermediatePercentage = gaitPercentage
		if(gaitPercentage<0.25){
			state= WalkingState.Rising
			gaitIntermediatePercentage=gaitPercentage*4.0
		}else if(gaitPercentage<0.5) {
			state= WalkingState.ToHome
			gaitIntermediatePercentage=(gaitPercentage-0.25)*4.0
		}else if(gaitPercentage<0.75) {
			state= WalkingState.ToNewTarget
			gaitIntermediatePercentage=(gaitPercentage-0.5)*4.0
		}else {
			state= WalkingState.Falling
			gaitIntermediatePercentage=(gaitPercentage-0.75)*4.0
		}
		
		
		
		if(!timout){
			double timeRemaining =(double) (System.currentTimeMillis()-reset)
			double percentage = timeRemaining/(double)(miliseconds)
			
		}
		
	}
	@Override
	public void DriveArc(MobileBase source, TransformNR newPose, double seconds) {
		DriveArcLocal( source,  newPose,  seconds,true);
	}
	/**
	 * Calc Inverse kinematics of a limb .
	 *
	 * @param jointSpaceVect the joint space vect
	 * @return the transform nr
	 */
	public double[] calcForward(DHParameterKinematics leg ,TransformNR transformTarget){
		return leg.inverseKinematics(leg.inverseOffset(transformTarget));
	}
	boolean check(DHParameterKinematics leg,TransformNR newPose){
		TransformNR stepup = newPose.copy();
		stepup.setZ(stepOverHeight + zLock );
		if(!leg.checkTaskSpaceTransform(newPose)){
			return false
		}
		if(!leg.checkTaskSpaceTransform(stepup)){
			return false
		}
		return true
	}
	
	private void loop(){

		
		
		walkingCycle()
		if(reset+3000 < System.currentTimeMillis()){
			println "FIRING reset from reset thread"
			resetting=true;
			long tmp= reset;
			if(home==null){
				home = new TransformNR[numlegs];
				for(int i=0;i<numlegs;i++){
					//home[i] = legs.get(i).forwardOffset(new TransformNR());
					home[i] =calcHome(legs.get(i))
				}
			}
			for(int i=0;i<numlegs;i++){
				TransformNR up = home[i].copy()
				up.setZ(stepOverHeight + zLock )
				TransformNR down = home[i].copy()
				down.setZ( zLock )
				try {
					// lift leg above home
					println "lift leg "+i+" "+up
					legs.get(i).setDesiredTaskSpaceTransform(up, 0);
				} catch (Exception e) {
					//println "Failed to reach "+up
					BowlerStudio.printStackTrace(e);
				}
				ThreadUtil.wait((int)stepOverTime);
				try {
					//step to new target
					println "step leg "+i+" down "+down
					
					legs.get(i).setDesiredTaskSpaceTransform(down, 0);
					//set new target for the coordinated motion step at the end
				} catch (Exception e) {
					//println "Failed to reach "+down
					BowlerStudio.printStackTrace(e);
				}
				ThreadUtil.wait((int)stepOverTime);
			}
			resettingindex=numlegs;
			resetting=false;
			threadDone=true;
			stepResetter=null;
			
		}
	}
	public void DriveArcLocal(MobileBase s, TransformNR n, double sec, boolean retry) {
		source=s;
		newPose=n.copy().inverse()
		miliseconds = Math.round(seconds*1000)

		if(stepResetter==null){
			timeOfCycleStart= System.currentTimeMillis();
			for(int i=0;i<numStepCycleGroups;i++){
				if(cycleGroups.get(i)==null){
					def cycleSet = []
					for(def leg:source.getLegs()){
						TransformNR  legRoot= leg.getRobotToFiducialTransform()
						if(legRoot.getX()>0&&legRoot.getY()>0 && i==0){
							cycleSet.add(leg)
						}else
						if(legRoot.getX()<0&&legRoot.getY()<0 && i==0){
							cycleSet.add(leg)
						}else
						if(legRoot.getX()>0&&legRoot.getY()<0 && i==1){
							cycleSet.add(leg)
						}else
						if(legRoot.getX()<0&&legRoot.getY()>0 && i==1){
							cycleSet.add(leg)
						}else{
							cycleSet.add(leg)//catch all 
						}
						
					}
					cycleGroups.put(i, cycleSet)
				}
			}
			
			stepResetter = new Thread(){
				public void run(){
					threadDone=false;
					state= Rising
					stepCycyleActiveIndex=0;
					println "Starting step reset thread"
					int numlegs = source.getLegs().size();
					 legs = source.getLegs();
					while(source.isAvailable() && threadDone==false){
						ThreadUtil.wait(10);
						loop()
					}
					println "Finished step reset thread"
				}
				
				
			};
			stepResetter.start();
		}
		resetStepTimer();
	}

	@Override
	public void DriveVelocityStraight(MobileBase source, double cmPerSecond) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void DriveVelocityArc(MobileBase source, double degreesPerSecond,
			double cmRadius) {
		// TODO Auto-generated method stub
		
	}



}

	
		