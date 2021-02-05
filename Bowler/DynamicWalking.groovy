package Bowler;

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




if(args==null){
	double stepOverHeight=15;
	long stepOverTime=400;// Servo loop times number of points times Nyquest doubeling
	Double zLock=5;
	Closure calcHome = { DHParameterKinematics leg -> 
			TransformNR h=leg.calcHome() 
	 		TransformNR  legRoot= leg.getRobotToFiducialTransform()
			TransformNR tr = leg.forwardOffset(new TransformNR())
			tr.setZ(zLock)
			//Bambi-on-ice the legs a bit

			if(legRoot.getY()>0){
				tr.translateY(10)
			}else{
				tr.translateY(-10)
			}

			if(legRoot.getX()>0){
				tr.translateX(10)
			}
			
			return tr;
	
	}
	boolean usePhysicsToMove = true;
	long stepCycleTime =600
	long walkingTimeout =stepCycleTime*2
	int numStepCycleGroups = 2
	
	double standardHeadTailAngle = -20
	double staticPanOffset = 20
	double coriolisGain = 1
	boolean headStable = false
	double maxBodyDisplacementPerStep = 30
	double minBodyDisplacementPerStep = 20
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
	minBodyDisplacementPerStep,
	walkingTimeout]
}

return new com.neuronrobotics.sdk.addons.kinematics.IDriveEngine (){
	double sitZ=-2;
    int Rising=0
    int ToHome=1
    int ToNewTarget=2
    int Falling=3
	boolean resetting=false;
	double stepOverHeight=(double)args.get(0);
	long stepOverTime=(long)args.get(1);
	private Double zLock=(Double)args.get(2);
	Closure calcHome =(Closure)args.get(3);
	boolean usePhysics=(args.size()>4?((boolean)args.get(4)):false);
	long stepCycleTime=args.get(5)
	long stepCycleTimeMax=args.get(5)
	long timeOfCycleStart= System.currentTimeMillis();
	int stepCycyleActiveIndex =0
	int numStepCycleGroups = args.get(6)
	double standardHeadTailAngle =args.get(7)
	double staticPanOffset = args.get(8)
	double coriolisGain=args.get(9)
	boolean headStable =args.get(10)
	double maxBodyDisplacementPerStep= args.get(11)
	double minBodyDisplacementPerStep =args.get(12)
	long walkingTimeout =args.get(13)
	
	ArrayList<DHParameterKinematics> legs;
	HashMap<Integer,ArrayList<DHParameterKinematics> > cycleGroups=new HashMap<>();
	HashMap<DHParameterKinematics,double[] > cycleStartPoint=new HashMap<>();
	TransformNR previousGLobalState;
	TransformNR target;
	TransformNR cachedNewPose;
	TransformNR cachedNewPoseStarting;
	double cachedSeconds;
	RotationNR rot;
	int resettingindex=0;
	private long reset = System.currentTimeMillis();
	
	Thread stepResetter=null;
	boolean threadDone=false
	int walkingState= Rising
	MobileBase source 
	TransformNR newPose =new TransformNR()
	long miliseconds
	boolean timout = false
	long loopTimingMS =5
	long timeOfLastLoop = System.currentTimeMillis()
	long timeOfLastIMUPrint = System.currentTimeMillis()
	int numlegs
	double gaitIntermediatePercentage 
	TransformNR global
	int coriolisIndex = 0
	double coriolisTimeBase =10.0
	double coriolisDivisions = 360.0/coriolisTimeBase
	double coriolisDivisionsScale = 360.0/coriolisDivisions

	long coriolisTimeLast=0
	double startAngle = 0
	double startSitz=0;
	double dynamicAngleX=0;
	double dynamicAngleY=0;
	public void resetStepTimer(){
		reset = System.currentTimeMillis();
	}
	def getUpLegs(){
		if(null!=cycleGroups.get(stepCycyleActiveIndex))
			return cycleGroups.get(stepCycyleActiveIndex)
		return []
	}
	def getDownLegs(){
		def vals= legs.collect{
				if(!getUpLegs().contains(it))
					return it
			}
		vals.removeAll([null])
		return vals
	}
	void updateDynamics(IMUUpdate update){
		if(stepResetter==null||reset+walkingTimeout < System.currentTimeMillis())
			return

			
		long incrementTime = (System.currentTimeMillis()-timeOfLastIMUPrint)
		double velocity=0
		if(	Math.abs(update.getRotxAcceleration())>0 ||
			Math.abs(update.getRotyAcceleration())>0 ||
			Math.abs(update.getzAcceleration())>0 
		){
			//println update.getxAcceleration()+" "+update.getzAcceleration()
			velocity =update.getzAcceleration()*1.5
			dynamicAngleX=update.getzAcceleration();
			dynamicAngleY=update.getyAcceleration();
			//velocity=update.getRotyAcceleration()
		}else
			velocity=0
		if(incrementTime>10){
			timeOfLastIMUPrint= System.currentTimeMillis()
			long timeSince=	(System.currentTimeMillis()-timeOfCycleStart)
			double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
			double gaitPercentage = gaitTimeRemaining/(double)(stepCycleTime)
			double sinPanOffset = Math.sin(gaitPercentage*Math.PI)*staticPanOffset
			
			double standardHeadTailPan = (stepResetter==null)?0:(stepCycyleActiveIndex%2==0?sinPanOffset:-sinPanOffset)
			double bobingPercent = Math.cos(gaitPercentage*Math.PI-Math.PI/2)*standardHeadTailAngle/2+standardHeadTailAngle/2
			double bobingPercentHead = Math.cos(gaitPercentage*Math.PI-Math.PI/2)*-42.8/2+-42.8/2
			if(stepResetter==null||reset+walkingTimeout < System.currentTimeMillis()){
				standardHeadTailPan=0;
				bobingPercent=0;
				bobingPercentHead=0;
			}
			for(def d:source.getAllDHChains()){
				String limbName = d.getScriptingName()
				double sinCop = Math.sin(Math.toRadians(coriolisIndex*coriolisDivisionsScale))
				double cosCop = Math.cos(Math.toRadians(coriolisIndex*coriolisDivisionsScale))
				
				//double computedTilt = bobingPercent+(velocity*sinCop*coriolisGain)
				//double computedTilHeadt = bobingPercentHead+(velocity*sinCop*coriolisGain)
				double computedTilt = (velocity*sinCop*coriolisGain)
				double computedTilHeadt = (velocity*sinCop*coriolisGain)
				double computedPan = standardHeadTailPan+(velocity*cosCop*coriolisGain)
				double computedPanHead = standardHeadTailPan+(velocity*-cosCop*coriolisGain)
				if(Math.abs(velocity)>5){
					//computedTilt = (velocity*sinCop*coriolisGain)
					//computedTilHeadt = (velocity*sinCop*coriolisGain)
					//computedPan = (velocity*cosCop*coriolisGain)
					//computedPanHead = (velocity*-cosCop*coriolisGain)
				}
				
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
					def l1 = d.getAbstractLink(0)
					def l2 = d.getAbstractLink(1)
					//
					if(computedTilt>l1.getMaxEngineeringUnits()){
						computedTilt=l1.getMaxEngineeringUnits();
					}
					if(computedTilt<l1.getMinEngineeringUnits()){
						computedTilt=l1.getMinEngineeringUnits();
					}
					//
					if(computedTilHeadt>l1.getMaxEngineeringUnits()){
						computedTilHeadt=l1.getMaxEngineeringUnits();
					}
					if(computedTilHeadt<l1.getMinEngineeringUnits()){
						computedTilHeadt=l1.getMinEngineeringUnits();
					}
					//
					if(computedPan>l2.getMaxEngineeringUnits()){
						computedPan=l2.getMaxEngineeringUnits();
					}
					if(computedPan<l2.getMinEngineeringUnits()){
						computedPan=l2.getMinEngineeringUnits();
					}
					//
					if(computedPanHead>l2.getMaxEngineeringUnits()){
						computedPanHead=l2.getMaxEngineeringUnits();
					}
					if(computedPanHead<l2.getMinEngineeringUnits()){
						computedPanHead=l2.getMinEngineeringUnits();
					}
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
										computedTilHeadt, //target angle
										0) // 2 seconds
							d.setDesiredJointAxisValue(1,// link index
										computedPanHead, //target angle
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
					//BowlerStudio.printStackTrace(e)
				}
			}

		}

	}
	void pose(def newAbsolutePose){
		try{
			def tipList = new HashMap<DHParameterKinematics,TransformNR >()
			for(def leg:legs){
				def home = dynamicHome(leg)
				//def home =leg.getCurrentTaskSpaceTransform()
				home.setZ( zLock )
				//tipList.put(leg,leg.getCurrentTaskSpaceTransform())
				tipList.put(leg,home)
			}
			source.setGlobalToFiducialTransform(newAbsolutePose)
			for(def leg:legs){
				def pose =tipList.get(leg)
				if(leg.checkTaskSpaceTransform(pose))
					leg.setDesiredTaskSpaceTransform(pose, 0);
			}
		}catch (Throwable t){
			BowlerStudio.printStackTrace(t)
		}
	}
	void sit(double sitAngle, double targetZ){

		if(!source.getScriptingName().contains("Kat"))
			return
		source.getImu().clearhardwareListeners()
				
		double incremnt = 0.05
		for(double i=0;i<1;i+=incremnt){
			double angle =  sitAngle*i+(startAngle*(1-i))
			double nowZ =  targetZ*i+(startSitz*(1-i))
			//println "Sitting to "+angle +" from "+startAngle
			def newTF =new TransformNR(0,
						  0, 
						  nowZ,
						  new RotationNR(0,
								  0, 
								angle
						  )
				      );
			pose(newTF)
			for(def d:source.getAllDHChains()){
				
				String limbName = d.getScriptingName()
				try{
					def headAngle = standardHeadTailAngle-angle/3
					def tailAngle=standardHeadTailAngle+angle/3
					def l1 = d.getAbstractLink(0)

					if(limbName.contentEquals("Tail")){
						if(tailAngle>l1.getMaxEngineeringUnits()){
							tailAngle=l1.getMaxEngineeringUnits();
						}
						if(tailAngle<l1.getMinEngineeringUnits()){
							tailAngle=l1.getMinEngineeringUnits();
						}
						d.setDesiredJointAxisValue(0,// link index
									tailAngle, //target angle
									0) // 2 seconds
						d.setDesiredJointAxisValue(1,// link index
									0, //target angle
									0) // 2 seconds
									
					} 
					if(limbName.contentEquals("Head")){
						if(headAngle>l1.getMaxEngineeringUnits()){
							headAngle=l1.getMaxEngineeringUnits();
						}
						if(headAngle<l1.getMinEngineeringUnits()){
							headAngle=l1.getMinEngineeringUnits();
						}
						d.setDesiredJointAxisValue(0,// link index
									headAngle, //target angle
									0) // 2 seconds
						d.setDesiredJointAxisValue(1,// link index
									0, //target angle
									0) // 2 seconds
									
					}
						
					//d.setDesiredTaskSpaceTransform(d.getCurrentTaskSpaceTransform(),  0);
					
				}catch(Exception e){
					//BowlerStudio.printStackTrace(e)
				}
			}
			Thread.sleep((long)(stepCycleTime*incremnt/source.getAllDHChains().size()))	
			
		}
		source.getImu().addhardwareListeners({update ->
			try{
				updateDynamics(update)
			}catch(Exception e){
				e.printStackTrace()
			}
		})
		
		startAngle=sitAngle
		startSitz=targetZ
	}
	private void walkLoop(){
		long incrementTime = (System.currentTimeMillis()-reset)
				if(incrementTime>miliseconds){
					timout = true
				}else
					timout = false		
		double tiltAngle = -1.0
		//println "Cycle = "+miliseconds+" "+incrementTime
			//if(stepCycyleActiveIndex%2==0)
		//	tiltAngle=tiltAngle*-1
		def zmove =tiltAngle*Math.sin(gaitPercentage*Math.PI)
		zmove=0
		if(Math.abs(dynamicAngleX)>1){
			def ymove = Math.sin(Math.toRadians(dynamicAngleX))*50
			ymove=0
			def angle = dynamicAngleX/4
			if(Math.abs(dynamicAngleX)<1){
				angle=0;
				ymove=0;
			}
			def tilt = new TransformNR(0,ymove,zmove,new RotationNR(angle,0,0))
			//pose(tilt)
			//source.setGlobalToFiducialTransform(tilt)
			//println "Moving y "+ymove+" angle "+angle
		}

		long time = System.currentTimeMillis()-timeOfLastLoop
		if(time>loopTimingMS){
			//print "\r\nWalk cycle loop time "+(System.currentTimeMillis()-timeOfLastLoop) +" "
			timeOfLastLoop=System.currentTimeMillis()
			walkingCycle()
			//print " Walk cycle took "+(System.currentTimeMillis()-timeOfLastLoop) 
		}
		if(reset+walkingTimeout< System.currentTimeMillis()){
			threadDone=true;
			stepResetter=null;	
			
		}
	}
	double getGaitPercentage(){
		double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
		return gaitTimeRemaining/(double)(stepCycleTime)
	}
	public void walkingCycle(){

		def upLegs = getUpLegs()
		def downLegs =getDownLegs()
		//println upLegs
		//println downLegs
		for (def leg :upLegs){
			if(leg!=null)
				upStateMachine(leg)
			
		}
		double gaitPercentage = getGaitPercentage()
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
		if(leg.checkTaskSpaceTransform(pose))
			leg.setDesiredTaskSpaceTransform(pose, 0);
	}
	private void upStateMachine(def leg){
		//println "Up Moving to "+percentage
		double gaitTimeRemaining = (double) (System.currentTimeMillis()-timeOfCycleStart)
		double gaitPercentage = gaitTimeRemaining/(double)(stepCycleTime)
		def tf =dynamicHome( leg)
		def NewTmpPose = timout?new TransformNR():newPose.inverse()


		
		def myPose=timout?new TransformNR():newPose

		switch(walkingState){
		case Rising:
			gaitIntermediatePercentage=gaitPercentage*4.0
			if(gaitIntermediatePercentage>1)
				gaitIntermediatePercentage=1
			//gaitIntermediatePercentage=1
			tf = compute(leg,gaitIntermediatePercentage,myPose)
			tf.setZ(zLock+(stepOverHeight*gaitIntermediatePercentage));
			if(gaitPercentage>0.25) {
				walkingState= ToHome
				//println "\nto Home " 
				getUpLegs().collect{
					if(it.checkTaskSpaceTransform(tf))
				 		cycleStartPoint.put(it,calcForward(it,tf))
				 	else
				 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
				//computeUpdatePose()
			}else
				break;
		case ToHome:
			gaitIntermediatePercentage=(gaitPercentage-0.25)*4.0
			if(gaitIntermediatePercentage>1)
				gaitIntermediatePercentage=1
			def current = compute(leg,0,myPose)
			
			def dyHome = dynamicHome( leg)
			//if(gaitIntermediatePercentage<0.9){
				double xinc=(dyHome.getX()-current.getX())*(1-gaitIntermediatePercentage);
				double yinc=(dyHome.getY()-current.getY())*(1-gaitIntermediatePercentage);
				
				dyHome.translateX(-xinc);
				dyHome.translateY(-yinc);
			//}

			tf=dyHome
			tf.setZ(zLock+(stepOverHeight));
			if(gaitPercentage>0.5) {
				//println "To new target " +gaitIntermediatePercentage
				walkingState= ToNewTarget
				getUpLegs().collect{
					if(it.checkTaskSpaceTransform(tf))
				 		cycleStartPoint.put(it,calcForward(it,tf))
				 	else
				 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
				//computeUpdatePose()
			}else
				break;
		case ToNewTarget:
			gaitIntermediatePercentage=(gaitPercentage-0.5)*4.0
			if(gaitIntermediatePercentage>1)
				gaitIntermediatePercentage=1
			double localPercent = gaitIntermediatePercentage*((double)numStepCycleGroups)
			//localPercent=((double)numStepCycleGroups-1)
			tf = compute(leg,localPercent,NewTmpPose)
			tf.setZ(zLock+(stepOverHeight));
			if(gaitPercentage>0.75) {
				walkingState= Falling
				//println "Falling " +gaitIntermediatePercentage
				getUpLegs().collect{
					if(it.checkTaskSpaceTransform(tf))
				 		cycleStartPoint.put(it,calcForward(it,tf))
				 	else
				 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
				//computeUpdatePose()
				
			}
			break;
		case Falling:
			gaitIntermediatePercentage=(gaitPercentage-0.75)*4.0
			if(gaitIntermediatePercentage>1)
				gaitIntermediatePercentage=1
			tf = compute(leg,gaitIntermediatePercentage,myPose)
			tf.setZ(zLock+stepOverHeight-(stepOverHeight*gaitIntermediatePercentage));
			//tf.setZ(zLock+stepOverHeight);
			if(gaitPercentage>1) {
				//tf = dynamicHome( leg)
				walkingState=Rising
				//print "\r\nRising Walk cycle loop time "+(System.currentTimeMillis()-timeOfCycleStart) +" "
				getUpLegs().collect{
					if(it.checkTaskSpaceTransform(tf))
				 		cycleStartPoint.put(it,calcForward(it,tf))
				 	else
				 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
				getDownLegs().collect{
					//def pose =compute(it,1,newPose)
					//if(it.checkTaskSpaceTransform(pose))
				 	//	cycleStartPoint.put(it,calcForward(it,pose))
				 	//else
				 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
				}
				timeOfCycleStart=System.currentTimeMillis()
				stepCycyleActiveIndex++
				if(stepCycyleActiveIndex==numStepCycleGroups){
					stepCycyleActiveIndex=0;
				}
				long start = System.currentTimeMillis()
				
				computeUpdatePose()
				//println "Compute new pose took : "+(System.currentTimeMillis()-start)
			}
			break;
		}
		//println "Gait Percent = "+gaitIntermediatePercentage
		if(leg.checkTaskSpaceTransform(tf))
			leg.setDesiredTaskSpaceTransform(tf, 0);
		else{
			if(leg.getScriptingName().contains("One")){
			//Log.enableErrorPrint()
			println leg.getScriptingName()+" failed in state "+ walkingState+" "+tf
			}
			
		}
	}
	private void computeUpdatePose(){
		if (cachedNewPose==null)
			return
		
		TransformNR n=cachedNewPose;
		double sec=cachedSecond
		cachedNewPose=null
		
		//n=new TransformNR()
		//stepCycleTime=Math.round(sec*1000)
		numlegs = source.getLegs().size();
		legs = source.getLegs();
		global= source.getFiducialToGlobalTransform();
		if(global==null){
			global=new TransformNR()
			
		}
		global=new TransformNR(global.getX(),
		  global.getY(), 
		  global.getZ(),
		  new RotationNR(Math.toDegrees(n.getRotation().getRotationTilt()),
				  Math.toDegrees(global.getRotation().getRotationAzimuth()), 
				 Math.toDegrees( n.getRotation().getRotationElevation())));
		n=new TransformNR(n.getX(),
		  n.getY(), 
		  n.getZ(),
		  new RotationNR(0,
				  Math.toDegrees(n.getRotation().getRotationAzimuth()), 
				  0));
		double percentOfPose=1
		double BodyDisplacement = Math.sqrt(Math.pow(n.getX(),2)+Math.pow(n.getY(),2))/1000
		double speedCalc = BodyDisplacement/sec
		double rotCalc = Math.toDegrees(n.getRotation().getRotationAzimuth())/sec
		stepCycleTime=Math.round(sec*percentOfPose*1000.0)
		if(stepCycleTime<stepOverTime){
			percentOfPose = ((double)stepOverTime)/((double)stepCycleTime)
		}
		try{
			newPose=scaleTransform(n,percentOfPose)
			stepCycleTime=Math.round(sec*percentOfPose*1000.0)
			//println "\n\nTarget at down target displacement = "+BodyDisplacement+" Absolute Velocity "+speedCalc+"m/s and  Z degrees per second= "+rotCalc+" cycle time = "+stepCycleTime
			
			double cycleMinimumDisplacement = minBodyDisplacementPerStep/(numStepCycleGroups)
			while(!newPosePossible(	newPose) &&
				percentOfPose>0.05 &&
				stepCycleTime<stepOverTime){
				percentOfPose-=0.1
				newPose=scaleTransform(n,percentOfPose)
				stepCycleTime=Math.round(sec*percentOfPose*1000.0)
				//println "Speeding up gait to meet speed "+stepCycleTime
			}
			while(newPosePossible(	newPose) &&
				stepCycleTime<stepOverTime){
				percentOfPose+=0.1
				newPose=scaleTransform(n,percentOfPose)
				stepCycleTime=Math.round(sec*percentOfPose*1000.0)
				//println "Speeding up gait to meet speed "+stepCycleTime
			}
			while(getMaximumDisplacement(newPose)< cycleMinimumDisplacement&& 
				stepCycleTime<stepCycleTimeMax&&
				newPosePossible(	newPose)
				){
				percentOfPose+=0.75
				stepCycleTime=Math.round(sec*percentOfPose*1000.0)
				newPose=scaleTransform(n,percentOfPose)
				//println "Slowing down up gait to meet speed "+stepCycleTime
			}
			while(!newPosePossible(	newPose) &&
				percentOfPose>0.01 ){
				percentOfPose-=0.1
				newPose=scaleTransform(n,percentOfPose)
				stepCycleTime=stepOverTime
				//println "Speeding up gait to meet speed "+stepCycleTime
			}
		}catch(Exception ex){
			newPose=new TransformNR()
		}

		if(!newPosePossible(	newPose)){
			Log.enableErrorPrint()
			println "\n\n\n\n!!!\nPose not possible\n "+newPose+"\n!!!!\n\n"
			newPose=new TransformNR()
		}

		miliseconds = Math.round(sec*percentOfPose*990)
		BodyDisplacement = Math.sqrt(Math.pow(newPose.getX(),2)+Math.pow(newPose.getY(),2))
		speedCalc = BodyDisplacement/((double)stepCycleTime)
		rotCalc = Math.toDegrees(newPose.getRotation().getRotationAzimuth())/((double)stepCycleTime)*1000.0
		println  String.format("Actual displacement = %.2f Moving at down target Absolute Velocity %.2f m/s and  Z degrees per second= %.2f cycle time = %d", BodyDisplacement,speedCalc,rotCalc,stepCycleTime);	
		
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
	private boolean newPosePossible(def newPose){
		for(def leg:legs){
			def ok=true
			def linkLocation = calcForward( leg , dynamicHome( leg))
			def linkLocationOld = cycleStartPoint.get(leg)
			double maxDownPercent = 	(numStepCycleGroups-1)
			cycleStartPoint.put(leg, linkLocation)	
			def pose =compute(leg,maxDownPercent,newPose)
			if(! leg.checkTaskSpaceTransform(pose))
				ok= false//one of the legs cant reach this pose
			pose.setZ(zLock+(stepOverHeight));
			if(! leg.checkTaskSpaceTransform(pose))
				ok= false//one of the legs cant reach this step over pose
			pose =compute(leg,maxDownPercent,newPose.inverse())
			if(! leg.checkTaskSpaceTransform(pose))
				ok= false//one of the legs cant reach this pose
			pose.setZ(zLock+(stepOverHeight));
			if(! leg.checkTaskSpaceTransform(pose))
				ok= false//one of the legs cant reach this step over pose
			cycleStartPoint.put(leg, linkLocationOld)	
			if(!ok)
				return false
		}
		return true;
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
	def dynamicHome(def leg){
		//TODO apply dynamics to home location
		return calcHome(leg).copy()
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

			cycleStartPoint.put(leg,leg.getCurrentJointSpaceVector())
				
		}catch(Exception e){
			BowlerStudio.printStackTrace(e)
		}
	}
	private void buildCycleGroups(){
		try{
			for(int i=0;i<numStepCycleGroups;i++){
				if(cycleGroups.get(i)==null){
					def cycleSet = []
					if(numStepCycleGroups==source.getLegs().size()){
						cycleSet.add(source.getLegs().get(i))
					}else if (numStepCycleGroups == 2) {
						int amount = (int)(source.getLegs().size()/numStepCycleGroups)
						if((amount%2)==0){
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
						}else{
							println "Alternating gait "
							for(int j=0;j<source.getLegs().size();j++){
								if(i==0){
									if((j%2)==0){
										cycleSet.add(source.getLegs().get(j))
									}
								}
								if(i==1){
									if((j%2)!=0){
										cycleSet.add(source.getLegs().get(j))
									}
								}
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
	}
	TransformNR scaleTransform(TransformNR incoming, double scale){
		return new TransformNR(incoming.getX()*scale,
			  incoming.getY()*scale, 
			  incoming.getZ()*scale,
			  new RotationNR(Math.toDegrees(incoming.getRotation().getRotationTilt())*scale,
					  Math.toDegrees(incoming.getRotation().getRotationAzimuth())*scale, 
					  Math.toDegrees(incoming.getRotation().getRotationElevation())*scale));
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
	@Override
	public void DriveArc(MobileBase source, TransformNR newPose, double seconds) {
		DriveArcLocal( source,  newPose,  seconds,true);
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
					try{
						updateDynamics(update)
					}catch(Exception e){
						e.printStackTrace()
					}
				})
				buildCycleGroups();
			}
			cachedNewPose=n;
			cachedNewPoseStarting=n;
			cachedSecond=sec;
			resetStepTimer();
		
			if(stepResetter==null){

				stepResetter = new Thread(){
					public void run(){
						computeUpdatePose()
						
						sit(0,0);
						legs.collect{
					 		cycleStartPoint.put(it,it.getCurrentJointSpaceVector())
						}
						timeOfCycleStart= System.currentTimeMillis();
						try{
							threadDone=false;
							walkingState= Rising
							stepCycyleActiveIndex=0;
							println "Starting step reset thread"
							timeOfCycleStart=System.currentTimeMillis()
							while(source.isAvailable() && stepResetter!=null){
								Thread.sleep(1)// avoid thread lock
								try{	
									walkLoop();
								}catch(Exception e){
									BowlerStudio.printStackTrace(e)
								}
							}
							cachedNewPose=new TransformNR();
							cachedNewPoseStarting=cachedNewPose;
							cachedSecond=sec;
							while(source.isAvailable() && getGaitPercentage()<0.99){// finish out the gait loop
								Thread.sleep(1)// avoid thread lock
								try{	
									walkLoop();
								}catch(Exception e){
									BowlerStudio.printStackTrace(e)
								}
							}
							
							
							println "Finished step reset thread"
							if(!source.getScriptingName().contains("Kat")){
								println "FIRING reset from reset thread"
								resetting=true;
								long tmp= reset;
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
								for(int i=0;i<numlegs;i++){
									StepHome(legs.get(i))
								}
								resettingindex=numlegs;
								resetting=false;
							}else
								sit(0,sitZ);
						}catch(Exception e){
							BowlerStudio.printStackTrace(e)
						}
					}
					
					
				};
				stepResetter.start();
			}
			
		}catch(Exception e){
			BowlerStudio.printStackTrace(e)
		}
	}

}

	
		
