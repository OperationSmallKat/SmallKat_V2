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
import com.neuronrobotics.sdk.addons.kinematics.imu.*

enum WalkingState {
    Rising,ToHome,ToNewTarget,Falling
}

if(args==null){
	double stepOverHeight=15;
	long stepOverTime=40;
	Double zLock=-12;
	Closure calcHome = { DHParameterKinematics leg -> 
			TransformNR h=leg.calcHome() 
	 		TransformNR  legRoot= leg.getRobotToFiducialTransform()
			TransformNR tr = leg.forwardOffset(new TransformNR())
			tr.setZ(zLock)
			//Bambi-on-ice the legs a bit
			if(legRoot.getY()>0){
				//tr.translateY(-5)
			}else{
				//tr.translateY(5)
			}
			
			return tr;
	
	}
	boolean usePhysicsToMove = true;
	long stepCycleTime =200
	int numStepCycleGroups = 2
	double standardHeadTailAngle = -25
	double staticPanOffset = 10
	args =  [stepOverHeight,stepOverTime,zLock,calcHome,usePhysicsToMove,stepCycleTime,numStepCycleGroups,standardHeadTailAngle,staticPanOffset]
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
	int numStepCycleGroups = args.get(6)
	double standardHeadTailAngle =args.get(7)
	double staticPanOffset = args.get(8)

	ArrayList<DHParameterKinematics> legs;
	HashMap<Integer,ArrayList<DHParameterKinematics> > cycleGroups=new HashMap<>();
	HashMap<DHParameterKinematics,double[] > cycleStartPoint=new HashMap<>();
	TransformNR previousGLobalState;
	TransformNR target;
	RotationNR rot;
	int resettingindex=0;
	private long reset = System.currentTimeMillis();
	
	Thread stepResetter=null;
	boolean threadDone=false
	WalkingState walkingState= WalkingState.Rising
	MobileBase source 
	TransformNR newPose
	long miliseconds
	boolean timout = true
	long loopTimingMS =5
	long timeOfLastLoop = System.currentTimeMillis()
	long timeOfLastIMUPrint = System.currentTimeMillis()
	int numlegs
	double gaitIntermediatePercentage 
	TransformNR global
	public void resetStepTimer(){
		reset = System.currentTimeMillis();
	}
	def getUpLegs(){
		return cycleGroups.get(stepCycyleActiveIndex)
	}
	def getDownLegs(){
		return legs.collect{
				if(!upLegs.contains(it))
					return it
			}
	}
	void updateDynamics(IMUUpdate update){
		
		long incrementTime = (System.currentTimeMillis()-timeOfLastIMUPrint)
		if(incrementTime>100){
			timeOfLastIMUPrint= System.currentTimeMillis()
			/*
			print "\r\nDynmics IMU state \n"
			for(def state :[update]){
				print " x = "+state.getxAcceleration()
				print "  y = "+state.getyAcceleration()
				print " z = "+state.getzAcceleration()
				print " rx = "+state.getRotxAcceleration()
				print " ry = "+state.getRotyAcceleration()
				print " rz = "+state.getRotzAcceleration()+"\r\n"
			}
			*/
		}
		
		double standardHeadTailPan = (stepResetter==null)?0:(stepCycyleActiveIndex==0?staticPanOffset:-staticPanOffset)
		for(def d:source.getAllDHChains()){
			String limbName = d.getScriptingName()
			double computedTilt = standardHeadTailAngle
			double computedPan = standardHeadTailPan
			if(limbName.contentEquals("Tail")){
				d.setDesiredJointAxisValue(0,// link index
							computedTilt, //target angle
							0) // 2 seconds
				d.setDesiredJointAxisValue(1,// link index
							computedPan, //target angle
							0) // 2 seconds			
			} 
			if(limbName.contentEquals("Head")){
				d.setDesiredJointAxisValue(0,// link index
							computedTilt, //target angle
							0) // 2 seconds
				d.setDesiredJointAxisValue(1,// link index
							computedPan, //target angle
							0) // 2 seconds			
			}
		}

	}
	public void walkingCycle(){
		
		long incrementTime = (System.currentTimeMillis()-reset)
		if(incrementTime>miliseconds){
			timout = true
		}else
			timout = false
		long timeSince=	(System.currentTimeMillis()-timeOfCycleStart)
		if(timeSince>stepCycleTime){
			//print "\r\nWalk cycle loop time "+(System.currentTimeMillis()-timeOfCycleStart) +" "
			legs.collect{
			 	cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
			}
			
			timeOfCycleStart=System.currentTimeMillis()
			stepCycyleActiveIndex++
			if(stepCycyleActiveIndex==numStepCycleGroups){
				stepCycyleActiveIndex=0;
			}
			walkingState=WalkingState.Rising
			
		}else{
			
			//println " Waiting till "+(timeOfCycleStart+stepCycleTime)+" is "+System.currentTimeMillis()leg
		}
		
		
		//println "Cycle = "+stepCycyleActiveIndex+" "+gaitPercentage
	
		def upLegs = getUpLegs()
		def downLegs =getDownLegs()
		//println upLegs
		//println downLegs
		for (def leg :upLegs){
			if(leg!=null)
				upStateMachine(leg)
			
		}
		double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
		double gaitPercentage = gaitTimeRemaining/(double)(stepCycleTime)
		if(!timout){
			double timeRemaining =(double) (System.currentTimeMillis()-reset)
			double percentage =timeRemaining/ (double)(miliseconds)
			//println "Walk Percent = "+percentage+" of " +miliseconds
			for (def leg :downLegs){
				if(leg!=null)
					downMove( leg, gaitPercentage)
			}
		}
		
		
	}
	private void downMove(def leg,double percentage){
		//println "Down Moving to "+percentage
		def pose =compute(leg,percentage,newPose)
		leg.setDesiredTaskSpaceTransform(pose, 0);
	}
	private def getLegCurrentPose(def leg){
		double[] joints = cycleStartPoint.get(leg)	
		TransformNR armOffset = leg.forwardKinematics(joints)	
		return leg.forwardOffset(armOffset);//leg.getCurrentTaskSpaceTransform();
	}
	private TransformNR compute(def leg,double percentage,def bodyMotion){
		double[] joints = cycleStartPoint.get(leg)	
		TransformNR armOffset = leg.forwardKinematics(joints)	
		TransformNR footStarting = leg.forwardOffset(armOffset);//leg.getCurrentTaskSpaceTransform();
		def myglobal=global.times(bodyMotion);// new global pose
		Matrix btt =  leg.getRobotToFiducialTransform().getMatrixTransform();
		Matrix ftb = myglobal.getMatrixTransform();// our new target
		Matrix current = armOffset.getMatrixTransform();
		Matrix mForward = ftb.times(btt).times(current);
		TransformNR inc =new TransformNR( mForward);
		inc.setZ(zLock);
		double xinc=(footStarting.getX()-inc.getX())*percentage;
		double yinc=(footStarting.getY()-inc.getY())*percentage;
		//apply the increment to the feet
		//println "Feet increments x = "+xinc+" y = "+yinc
		footStarting.translateX(xinc);
		footStarting.translateY(yinc);
		footStarting.setZ(zLock);
		
		return footStarting
	}
	private void upStateMachine(def leg){
		//println "Up Moving to "+percentage
		double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
		double gaitPercentage = gaitTimeRemaining/(double)(stepCycleTime)
		def tf = getLegCurrentPose(leg)
		def NewTmpPose = newPose.inverse()
		if(timout)
			NewTmpPose=new TransformNR()
		switch(walkingState){
		case WalkingState.Rising:
			gaitIntermediatePercentage=gaitPercentage*4.0
			tf = compute(leg,gaitIntermediatePercentage,newPose)
			tf.setZ(zLock+(stepOverHeight*gaitIntermediatePercentage));
			if(gaitPercentage>0.25) {
				walkingState= WalkingState.ToHome
				//println "to Home " 
				getUpLegs().collect{
					if(it!=null)
				 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
			}
			break;
		case WalkingState.ToHome:
			gaitIntermediatePercentage=(gaitPercentage-0.25)*4.0
			def current = tf
			tf = dynamicHome( leg)
			if(gaitIntermediatePercentage<0.9){
				double xinc=(tf.getX()-current.getX())*(1-gaitIntermediatePercentage);
				double yinc=(tf.getY()-current.getY())*(1-gaitIntermediatePercentage);
				
				tf.translateX(-xinc);
				tf.translateY(-yinc);
			}
			tf.setZ(zLock+(stepOverHeight));
			if(gaitPercentage>0.5) {
				//println "To new target " +gaitIntermediatePercentage
				walkingState= WalkingState.ToNewTarget
				getUpLegs().collect{if(it!=null)
				 	cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
			}
			break;
		case WalkingState.ToNewTarget:
			gaitIntermediatePercentage=(gaitPercentage-0.5)*4.0
			tf = compute(leg,gaitIntermediatePercentage,NewTmpPose)
			tf.setZ(zLock+(stepOverHeight));
			if(gaitPercentage>0.75) {
				walkingState= WalkingState.Falling
				//println "Falling " +gaitIntermediatePercentage
				getUpLegs().collect{if(it!=null)
				 	cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
			}
			break;
		case WalkingState.Falling:
			gaitIntermediatePercentage=(gaitPercentage-0.75)*4.0
			tf = compute(leg,1-gaitIntermediatePercentage,NewTmpPose)
			tf.setZ(zLock+stepOverHeight-(stepOverHeight*gaitIntermediatePercentage));
			break;
		}
		//println "Gait Percent = "+gaitIntermediatePercentage
		
		leg.setDesiredTaskSpaceTransform(tf, 0);
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
	
	private void walkLoop(){
		long time = System.currentTimeMillis()-timeOfLastLoop
		if(time>loopTimingMS){
			//print "\r\nWalk cycle loop time "+(System.currentTimeMillis()-timeOfLastLoop) +" "
			timeOfLastLoop=System.currentTimeMillis()
			walkingCycle()
			//print " Walk cycle took "+(System.currentTimeMillis()-timeOfLastLoop) 
		}
		if(reset+5000 < System.currentTimeMillis()){
			println "FIRING reset from reset thread"
			resetting=true;
			long tmp= reset;
			
			for(int i=0;i<numlegs;i++){
				StepHome(legs.get(i))
			}
			resettingindex=numlegs;
			resetting=false;
			threadDone=true;
			stepResetter=null;
			
		}
	}
	def dynamicHome(def leg){
		//TODO apply dynamics to home location
		return calcHome(leg)
	}
	private void StepHome(def leg){
		try{
			def home = dynamicHome(leg)
			TransformNR up = home.copy()
			up.setZ(stepOverHeight + zLock )
			TransformNR down = home.copy()
			down.setZ( zLock )
			try {
				// lift leg above home
				//println "lift leg  "+up
				leg.setDesiredTaskSpaceTransform(up, 0);
			} catch (Exception e) {
				//println "Failed to reach "+up
				BowlerStudio.printStackTrace(e);
			}
			ThreadUtil.wait((int)stepOverTime);
			try {
				//step to new target
				//println "step leg down "+down
				
				leg.setDesiredTaskSpaceTransform(down, 0);
				//set new target for the coordinated motion step at the end
			} catch (Exception e) {
				//println "Failed to reach "+down
				BowlerStudio.printStackTrace(e);
			}
			ThreadUtil.wait((int)stepOverTime);
		}catch(Exception e){
			BowlerStudio.printStackTrace(e)
		}
	}
	public void DriveArcLocal(MobileBase s, TransformNR n, double sec, boolean retry) {
		try{
			//println "Walk update "+n
			if(s==null){
				println "Null mobile base"
				return
			}
			if(source!=s){
				source=s;
				source.getImu().clearhardwareListeners()
				source.getImu().addhardwareListeners({update ->
					updateDynamics(update)
				})
			}
			newPose=n.copy()
			//newPose=new TransformNR()
			miliseconds = Math.round(sec*1000)
			numlegs = source.getLegs().size();
			legs = source.getLegs();
			global= source.getFiducialToGlobalTransform();
			if(global==null){
				global=new TransformNR()
				source.setGlobalToFiducialTransform(global)
			}
			if(stepResetter==null){
				try{
					timeOfCycleStart= System.currentTimeMillis();
					for(int i=0;i<numStepCycleGroups;i++){
						if(cycleGroups.get(i)==null){
							
							def cycleSet = []
							if(numStepCycleGroups==source.getLegs().size()){
								cycleSet.add(source.getLegs().get(i))
							}else if (numStepCycleGroups == 2) {
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
									}
								}
							}
							//println "Adding "+cycleSet.size()+" to index "+i
							cycleGroups.put(i, cycleSet)
						}
					}
				}catch(Exception e){
					BowlerStudio.printStackTrace(e)
				}
				stepResetter = new Thread(){
					public void run(){
						try{
							threadDone=false;
							walkingState= WalkingState.Rising
							stepCycyleActiveIndex=0;
							println "Starting step reset thread"
							while(source.isAvailable() && threadDone==false){
								Thread.sleep(0,10)// avoid thread lock
								try{	
									walkLoop();
								}catch(Exception e){
									BowlerStudio.printStackTrace(e)
								}
							}
							println "Finished step reset thread"
						}catch(Exception e){
							BowlerStudio.printStackTrace(e)
						}
					}
					
					
				};
				stepResetter.start();
			}
			resetStepTimer();
		}catch(Exception e){
			BowlerStudio.printStackTrace(e)
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

	
		