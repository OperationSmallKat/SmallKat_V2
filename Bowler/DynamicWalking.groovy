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
	Double zLock=0;
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
	long stepCycleTime =5000
	int numStepCycleGroups = 2
	double standardHeadTailAngle = -20
	double staticPanOffset = 10
	double coriolisGain = 1
	boolean headStable = false
	double maxBodyDisplacementPerStep = 30
	double minBodyDisplacementPerStep = 2
	args =  [stepOverHeight,
	stepOverTime,
	zLock,
	calcHome,
	usePhysicsToMove,
	stepCycleTime,
	numStepCycleGroups,
	standardHeadTailAngle,
	staticPanOffset,
	coriolisGain,
	headStable,
	maxBodyDisplacementPerStep,
	minBodyDisplacementPerStep]
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
	double coriolisGain=args.get(9)
	boolean headStable =args.get(10)
	double maxBodyDisplacementPerStep= args.get(11)
	double minBodyDisplacementPerStep =args.get(12)

	
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
	int coriolisIndex = 0
	double coriolisDivisions = 360.0
	double coriolisTimeBase = 500.0
	long coriolisTimeLast=0
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
		if(stepResetter==null)
			return
		long incrementTime = (System.currentTimeMillis()-timeOfLastIMUPrint)
		double velocity=0
		if(	Math.abs(update.getxAcceleration())>0 ||
			Math.abs(update.getxAcceleration())>0 ||
			Math.abs(update.getxAcceleration())>0 
		)
			velocity=update.getRotyAcceleration()
		else
			velocity=0
		if(incrementTime>100){
			timeOfLastIMUPrint= System.currentTimeMillis()
			/*
			print "\r\nDynmics IMU state \n"
			for(def state :[update]){
				print " x = "+update.getxAcceleration()
				print "  y = "+update.getyAcceleration()
				print " z = "+update.getzAcceleration()
				print " rx = "+update.getRotxAcceleration()
				print " ry = "+update.getRotyAcceleration()
				print " rz = "+update.getRotzAcceleration()+"\r\n"
			}
			*/
		}
		long timeSince=	(System.currentTimeMillis()-timeOfCycleStart)
		double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
		double gaitPercentage = gaitTimeRemaining/(double)(stepCycleTime)
		double sinPanOffset = Math.sin(gaitPercentage*Math.PI)*staticPanOffset
		
		double standardHeadTailPan = (stepResetter==null)?0:(stepCycyleActiveIndex==0?sinPanOffset:-sinPanOffset)
		double bobingPercent = Math.cos(gaitPercentage*Math.PI-Math.PI/2)*standardHeadTailAngle/2+standardHeadTailAngle/2
		for(def d:source.getAllDHChains()){
			String limbName = d.getScriptingName()
			double sinCop = Math.sin(Math.toRadians(coriolisIndex))
			double cosCop = Math.cos(Math.toRadians(coriolisIndex))
			
			double computedTilt = bobingPercent+(velocity*sinCop*coriolisGain)
			double computedPan = standardHeadTailPan+(velocity*cosCop*coriolisGain)
			long coriolisincrementTime = (System.currentTimeMillis()-coriolisTimeLast)
			double coriolisTimeDivisionIncrement = (coriolisTimeBase/coriolisDivisions)
			//println coriolisIndex+" Time division = "+coriolisTimeDivisionIncrement+" elapsed = "+coriolisincrementTime
			if(coriolisTimeDivisionIncrement<coriolisincrementTime){
				coriolisTimeLast=System.currentTimeMillis()
				if(velocity>0){
					coriolisIndex++;
					coriolisIndex=(coriolisIndex>=coriolisDivisions?0:coriolisIndex)
				}else{
					coriolisIndex--;
					coriolisIndex=(coriolisIndex<0?coriolisDivisions-1:coriolisIndex)
				}
			}
			try{
			if(limbName.contentEquals("Tail")){
				d.setDesiredJointAxisValue(0,// link index
							computedTilt, //target angle
							0) // 2 seconds
				d.setDesiredJointAxisValue(1,// link index
							computedPan, //target angle
							0) // 2 seconds			
			} 
			if(limbName.contentEquals("Head")){
				if(!headStable){
					d.setDesiredJointAxisValue(0,// link index
								computedTilt, //target angle
								0) // 2 seconds
					d.setDesiredJointAxisValue(1,// link index
								computedPan, //target angle
								0) // 2 seconds
				}else{
					d.setDesiredJointAxisValue(0,// link index
								standardHeadTailAngle, //target angle
								0) // 2 seconds
					d.setDesiredJointAxisValue(1,// link index
								0, //target angle
								0) // 2 seconds				
				}
			}
			}catch(Exception e){
				BowlerStudio.printStackTrace(e)
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
			//double percentage =timeRemaining/ (double)(miliseconds)
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
		def vals = getDisplacementIncrement(leg,bodyMotion)
		TransformNR footStarting=vals[2]
		//apply the increment to the feet
		//println "Feet increments x = "+xinc+" y = "+yinc
		footStarting.translateX(vals[0]*percentage);
		footStarting.translateY(vals[1]*percentage);
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
			for(def d:source.getAllDHChains()){
				String limbName = d.getScriptingName()
				try{
					if(limbName.contentEquals("Tail")){
						d.setDesiredJointAxisValue(0,// link index
									standardHeadTailAngle, //target angle
									0) // 2 seconds
						d.setDesiredJointAxisValue(1,// link index
									0, //target angle
									0) // 2 seconds			
					} 
					if(limbName.contentEquals("Head")){
						d.setDesiredJointAxisValue(0,// link index
									standardHeadTailAngle, //target angle
									0) // 2 seconds
						d.setDesiredJointAxisValue(1,// link index
									0, //target angle
									0) // 2 seconds			
					}
					coriolisIndex=0;
				}catch(Exception e){
					BowlerStudio.printStackTrace(e)
				}
			}
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
			
			//newPose=new TransformNR()
			miliseconds = Math.round(sec*1000)
			//stepCycleTime=Math.round(sec*1000)
			numlegs = source.getLegs().size();
			legs = source.getLegs();
			global= source.getFiducialToGlobalTransform();
			double timescaleing = ((double)stepCycleTime)/(sec*1000.0)
			newPose=scaleTransform(n,timescaleing)
			// Compute the incremental transform size
			
			while(getMaximumDisplacement(newPose)>maxBodyDisplacementPerStep && stepCycleTime>100){
				stepCycleTime-=10
				timescaleing = ((double)stepCycleTime)/(sec*1000.0)
				newPose=scaleTransform(n,timescaleing)
				println "Speeding up gait to meet speed "+stepCycleTime
			}
			while(getMaximumDisplacement(newPose)>minBodyDisplacementPerStep && stepCycleTime<1000){
				stepCycleTime+=10
				timescaleing = ((double)stepCycleTime)/(sec*1000.0)
				newPose=scaleTransform(n,timescaleing)
				println "Speeding up gait to meet speed "+stepCycleTime
			}
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
	TransformNR scaleTransform(TransformNR incoming, double scale){
		return new TransformNR(incoming.getX()*scale,
			  incoming.getY()*scale, 
			  incoming.getZ()*scale,
			  new RotationNR(incoming.getRotation().getRotationTilt()*scale,
					  incoming.getRotation().getRotationAzimuth()*scale, 
					  incoming.getRotation().getRotationElevation()*scale));
	}
	double getMaximumDisplacement(TransformNR bodyMotion){
		double max=0;
		for(def leg:legs){
			def disp =getDisplacement( leg, bodyMotion) 
			if(Math.abs(disp)>max){
				max=Math.abs(disp)
			}
		}
		return max
	}
	
	double getDisplacement(def leg,TransformNR bodyMotion){
		def vals = getDisplacementIncrement(leg,bodyMotion)

		return Math.sqrt(Math.pow(vals[0],2)+Math.pow(vals[1],2))
	}
	def getDisplacementIncrement(def leg,TransformNR bodyMotion){
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
		double xinc=(footStarting.getX()-inc.getX());
		double yinc=(footStarting.getY()-inc.getY());
		return [xinc,yinc,footStarting]
	}
	double getDeltaRotation(TransformNR incoming){
		
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

	
		