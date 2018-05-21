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
	
	public void resetStepTimer(){
		reset = System.currentTimeMillis();
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
	
	private void resetLoop(){
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
	public void DriveArcLocal(MobileBase source, TransformNR newPose, double seconds, boolean retry) {
		TransformNR incomingTarget=newPose.copy()
		newPose = newPose.inverse()
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
					println "Starting step reset thread"
					int numlegs = source.getLegs().size();
					 legs = source.getLegs();
					while(source.isAvailable() && threadDone==false){
						ThreadUtil.wait(10);
						resetLoop()
					}
					println "Finished step reset thread"
				}
				
				
			};
			stepResetter.start();
		}
		resetStepTimer();
		
	
		try{
				 numlegs = source.getLegs().size();
				TransformNR [] feetLocations = new TransformNR[numlegs];
				TransformNR [] newFeetLocations = new TransformNR[numlegs];
				ArrayList<DHParameterKinematics> legs = source.getLegs();
				
				if(home==null){
					Log.enableSystemPrint(true)
					home = new TransformNR[numlegs];
					for(int i=0;i<numlegs;i++){
						//home[i] = legs.get(i).forwardOffset(new TransformNR());
						home[i] =calcHome(legs.get(i))
						println "Home for link "+i+" is "+home[i]
					}
				}

				//println "Loading feet target locations for "+legs.size()
				// Load in the locations of the tips of each of the feet.
				for(int i=0;i<legs.size();i++){
					//println "Loading Leg "+legs.get(i).getScriptingName()
					def leg = legs.get(i)
					TransformNR global= source.getFiducialToGlobalTransform();
					if(global==null){
						global=new TransformNR()
						source.setGlobalToFiducialTransform(global)
					}
					TransformNR footStarting = leg.getCurrentTaskSpaceTransform();

					if(global==null)
						global=new TransformNR()
					double[] joints = leg.getCurrentJointSpaceVector()	
					TransformNR armOffset = leg.forwardKinematics(joints)	
					global=global.times(newPose);// new global pose
					Matrix btt =  leg.getRobotToFiducialTransform().getMatrixTransform();
					Matrix ftb = global.getMatrixTransform();// our new target
					Matrix current = armOffset.getMatrixTransform();
					Matrix mForward = ftb.times(btt).times(current);
					TransformNR inc =new TransformNR( mForward);
					feetLocations[i]=inc
					
					if(zLock==null){
						//sets a standard plane at the z location of the first leg.
						zLock=feetLocations[i].getZ();
						//println "ZLock level set to "+zLock
					}
					home[i] =calcHome(leg)
					feetLocations[i].setZ(home[i].getZ());


				}
				boolean resetDetect=false;
				for(int i=0;i<numlegs;i++){
					double footx,footy;
					newFeetLocations[i]=legs.get(i).getCurrentPoseTarget();
					// start by storing where the feet are
					DHParameterKinematics leg=legs.get(i)
					if( !check(leg,feetLocations[i])||
						!check(leg,newFeetLocations[i])
					){
						//perform the step over
						home[i] =calcHome(leg)
						//println "Leg "+i+" setep over to x="+feetLocations[i].getX()+" y="+feetLocations[i].getY()

						//println i+" foot reset needed "+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
						
						//Force the search for a new foothold to start at the home point
						feetLocations[i].setX(home[i].getX());
						feetLocations[i].setY(home[i].getY());
						//feetLocations[i]=newFeetLocations[i].copy()
						feetLocations[i].setZ(zLock);
						int j=0;
						//println i+" Step over location"+feetLocations[i].getX()+" y:"+feetLocations[i].getY()
						
						double xunit;
						double yunit ;
						TransformNR lastGood= feetLocations[i].copy();
						TransformNR stepup = feetLocations[i].copy();
						TransformNR stepUnit = feetLocations[i].copy();
						stepup.setZ(stepOverHeight + zLock );
						/*
						 * legs.get(i).checkTaskSpaceTransform(feetLocations[i]) &&
							 legs.get(i).checkTaskSpaceTransform(stepup) &&
							 legs.get(i).checkTaskSpaceTransform(stepUnit) &&
						 */
						ArrayList<TransformNR> stepOverTrajectory =[]
					
						
						while(j<15){
							feetLocations[i].setZ(zLock );
							stepUnit=lastGood;
							lastGood=feetLocations[i].copy();
							TransformNR g= source.getFiducialToGlobalTransform();
							if(g==null)
								g=new TransformNR()
							g=g.times(newPose);// new global pose
							double[] joints = legs.get(i).inverseKinematics(legs.get(i).inverseOffset(feetLocations[i]))	
							TransformNR armOffset = legs.get(i).forwardKinematics(joints)
							Matrix btt = legs.get(i).getRobotToFiducialTransform().getMatrixTransform();
							Matrix ftb = g.getMatrixTransform();// our new target
							Matrix current = armOffset.getMatrixTransform();
							Matrix mForward = ftb.times(btt).times(current);
							TransformNR incr =new TransformNR( mForward);
							//now calculate a a unit vector increment
							double xinc=(feetLocations[i].getX()-incr.getX())/1;
							double yinc=(feetLocations[i].getY()-incr.getY())/1;
							//apply the increment to the feet
							feetLocations[i].translateX(xinc);
							feetLocations[i].translateY(yinc);
							//feetLocations[i].translateX(-newPose.getX());
							//feetLocations[i].translateY(-newPose.getX());
							j++;
							stepup = feetLocations[i].copy()
							stepup.setZ(stepOverHeight + zLock );
							if(!check(leg,feetLocations[i])){
								lastGood.setZ(zLock );
								feetLocations[i]=lastGood.copy()
								stepOverTrajectory.add(feetLocations[i])
								break
							}
							lastGood=feetLocations[i]
							stepOverTrajectory.add(stepup.copy())
						}
						double diffx= newFeetLocations[i].getX() - home[i].getX()
						double diffy= newFeetLocations[i].getY() - home[i].getY()
						TransformNR stepupFirst = newFeetLocations[i].copy()
						stepupFirst.setZ(stepOverHeight +zLock);
						double numSteps=stepOverTrajectory.size()
						for(int x=0;x<numSteps;x++){
							stepupFirst.translateX(-diffx/numSteps);
							stepupFirst.translateY(-diffy/numSteps);
							stepOverTrajectory.add(x,stepupFirst.copy())
						}
						resetting=true;
						resettingindex=i;
						//println "Resetting with "+stepOverTrajectory.size()+" points"
						for(int x=0;x<stepOverTrajectory.size();x++){	
							double time = stepOverTime/stepOverTrajectory.size()	
							ThreadUtil.wait((int)(time));		
							try {
								//println stepOverTrajectory.get(x)
								leg.setDesiredTaskSpaceTransform(stepOverTrajectory.get(x), time/1000.0);
							} catch (Exception e) {
								//println "Failed to reach "+stepup
								e.printStackTrace();
							}
							
						}
						resetting=false;
						resettingindex=numlegs;
						resetDetect=true;

						
					}
					
					resetStepTimer();
				}
				if(resetDetect && retry){
					 DriveArcLocal( source,  incomingTarget,  seconds,false) 
					 return;
				}
					
				for(int i=0;i<numlegs;i++){
					if(!legs.get(i).checkTaskSpaceTransform(feetLocations[i])){
						throw new RuntimeException(i + " leg foot location is not acheivable "+newPose);
					}
					
				}
				
				
				//all legs have a valid target set, perform coordinated motion
				for(int i=0;i<numlegs;i++){
					legs.get(i).setDesiredTaskSpaceTransform(feetLocations[i], seconds*1.5);
		
				}
				if(!usePhysics)
					source.setGlobalToFiducialTransform(global);
				
				// while(resetting && source.isAvailable()){
				// 	//System.out.println("Waiting...")
				// 	ThreadUtil.wait(100);
				// }
				
		}catch (Exception ex){
			ex.printStackTrace(System.out);
			println "This step is not possible, undoing "+newPose
			
			//Set it back to where it was to use the interpolator for global move at the end
			throw ex
			
		}
		
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
