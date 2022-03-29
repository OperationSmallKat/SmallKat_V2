package Bowler;

import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.Log

IDriveEngine engine = new IDriveEngine () {
	/**
	* Driving kinematics should be implemented in here
	*
	*
	* This method should not block You will get that called every 0.1 to 0.01 seconds
	* by the jog widget with a small displacement transform. If the last command
	* finishes before a new one comes in, reset the driving device.
	* if a new command comes in then keep moving. Assume that the most important
	* thing here is time synchronicity.you may get it called with a large transform,
	* larger than you can take in one step,a nd you may get a transform with a step size so
	* small it would never move. You will need to warp and stretch the transform coming in
	* to make sure there are an integer number of steps, with at least some minimum step length.
	*
	* Be sure to have any threads you create timeout and die, don't wait for disconnect, as you
	* are developing that will be a pain in the ass
	*
	* Essentially, this command defines a velocity (transform/second)and you need to maintain
	* that velocity, descretized into steps, and stop as soon as the last velocity term times out
	* also, do not assume it will ever be pure rotation nor pure translation, assume all
	* commands are a combo of both.
	*
	* @param source the source
	* @param newPose the new pose that should be achived.
	* @param seconds how many seconds it should take
	*/

	boolean firstRun=true
	double zoffsetOfFeetHome = -12.5
	double xOffsetOfFeetHome = 5
	double ySplayOut = 10
	double stepOverHeight = 7
	public void DriveArc(MobileBase source,TransformNR newPose,double seconds) {
		try {
		
			def con = DeviceManager.getSpecificDevice("BodyController-"+source.getScriptingName(),{
				BodyController bc= new BodyController()
				bc.connect();
				source.setHomeProvider({limb->
					return limb.calcHome()
								.translateZ(zoffsetOfFeetHome)// move the starting point down
								.translateX(xOffsetOfFeetHome) // move the starting point forward
								.translateY(limb.getRobotToFiducialTransform().getY()>0?
									ySplayOut:-ySplayOut) // move the starting point forward
								
				})
				return bc;
			})
			con.stepOverHeight=stepOverHeight
			con.source=source
			con.incomingPose=newPose
			con.incomingSeconds=seconds
			con.timeOfMostRecentCommand=System.currentTimeMillis()
			
		}catch(Throwable t) {
			t.printStackTrace()
		}
	}
}

return engine

enum CycleState {
	waiting,
	cycleStart,
	stepThroughPoints,
	checkForContinue,
	cycleFinish
}
class BodyController{
	int numberOfInterpolationPoints=0
	double stepOverHeight =1

	double numPointsInLoop =12.0
	double unitScale =1.0/(numPointsInLoop/2.0)
	Thread bodyLoop = null;
	boolean availible=true;
	int numMsOfLoop = 32;
	int coriolisIndex = 0
	// ms of the tail loop
	double timeOfTailLoop = numMsOfLoop*10
	double coriolisTimeBase =numMsOfLoop
	// degrees per time slice
	double coriolisDivisions = timeOfTailLoop/coriolisTimeBase
	double coriolisDivisionsScale = 360.0/coriolisDivisions
	double coriolisGain=3.5
	
	double cycleTime = numMsOfLoop*numPointsInLoop*(numberOfInterpolationPoints+1)+(numMsOfLoop*1)
	int numberOfInterpolatedPointsInALoop = numPointsInLoop*(numberOfInterpolationPoints+1)
	MobileBase source=null;
	MobileBase lastSource=null
	TransformNR newPose=null;
	TransformNR incomingPose=null;
	double incomingSeconds=0;
	double seconds=0;
	long timeOfMostRecentCommand=0;
	boolean cycleStarted=false;
	TransformNR measuredPose=null ;
	def state=CycleState.waiting;
	int pontsIndex=0;
	DHParameterKinematics head=null;
	DHParameterKinematics tail=null;
	HashMap<DHParameterKinematics,ArrayList<TransformNR>> legTipMap=null;
	long timeOfStartOfCycle = System.currentTimeMillis()
	double tailDefaultLift=0;
	
	private void loop() {
		
		double timeElapsedSinceLastCommand = ((double)(System.currentTimeMillis()-timeOfMostRecentCommand))/1000.0
		switch(state) {
			case CycleState.waiting:
				if(source==null) {
//					if(lastSource!=null && measuredPose!=null) {
//						lastSource.setGlobalToFiducialTransform(measuredPose)
//					}
					break;
				}else {
				
					if(lastSource!=null) {
						// remove old hardware listeners
						lastSource.setGlobalToFiducialTransform(new TransformNR())
						lastSource.getImu().clearhardwareListeners()
					}
					lastSource=source;
					// Add the IMU listener
					source.getImu().addhardwareListeners({update ->
						// read the IMU and transform it to match the hardware
						measuredPose=new TransformNR(0,0,0,new RotationNR(	-update.getxAcceleration(),
								update.getyAcceleration()-90,	update.getzAcceleration()	))
									.times(new TransformNR(0,0,0,new RotationNR(180,0,0)))
									.times(new TransformNR(0,0,0,new RotationNR(0,90,0)))
					})
					measuredPose=new TransformNR();
					// Search for the head and tail limbs
					for(DHParameterKinematics d:source.getAllDHChains()) {
						if(d.getScriptingName().contentEquals("Tail")) {
							tail=d;
							for(int i=0;i<d.numberOfLinks;i++) {
								// disable link limit exceptions
								tail.getAbstractLink(i).setUseLimits(false);
							}
						}
						if(d.getScriptingName().contentEquals("Head")) {
							head=d;
							for(int i=0;i<d.numberOfLinks;i++) {
								// disable link limit exceptions
								head.getAbstractLink(i).setUseLimits(false);
							}
						}
					}
					tailDefaultLift = tail.getAbstractLink(0).getMinEngineeringUnits()
				}
				state=CycleState.cycleStart;
				
			//no break
			case CycleState.cycleStart:
				//println "Walk cycle starting "+cycleTime + " last Took "+(System.currentTimeMillis()-timeOfStartOfCycle);
				timeOfStartOfCycle = System.currentTimeMillis()
				pontsIndex=0;
				newPose=incomingPose
				seconds=incomingSeconds
				setupCycle()
				state=CycleState.stepThroughPoints
			//no break
			case CycleState.stepThroughPoints:
				if(pontsIndex<numberOfInterpolatedPointsInALoop) {
					//println "Interpolation point "+pontsIndex+" time elapsed "+timeElapsedSinceLastCommand
					doStep()
					pontsIndex++;
					break;
				}else {
					state=CycleState.checkForContinue
				}
				//no break
			case CycleState.checkForContinue:
				if((timeElapsedSinceLastCommand)<seconds/2) {
					state=CycleState.cycleStart
					//println "Cycle not finished, stepping again took "+timeElapsedSinceLastCommand+" expected "+seconds
					break;
				}else {
					state=CycleState.cycleFinish
				}
				// no break
			case CycleState.cycleFinish:
				doFinishingMove()
				state=CycleState.waiting
				//println "Finising move"
				source=null
				clearLegTips()
				break;
		}
	}

	private void runDynamics() {
		if(measuredPose!=null) {
			double tiltAngle = Math.toDegrees(measuredPose.getRotation().getRotationTilt())
			if(tiltAngle>90)
				tiltAngle+=-180
			if(tiltAngle<-90)
				tiltAngle+=180
			def abs = Math.abs(tiltAngle)
			def min =5
			if(abs<min) {
				coriolisIndex=0;
				abs=min
			}
			def coriolisIndexCoriolisDivisionsScale = coriolisIndex*coriolisDivisionsScale
			
			def coriolisIndexCoriolisDivisionsScaleTiltAngle = coriolisIndexCoriolisDivisionsScale+tiltAngle
			//println "Measured tilt = "+tiltAngle+" target = "+coriolisIndexCoriolisDivisionsScaleTiltAngle
			double sinCop = Math.sin(Math.toRadians(coriolisIndexCoriolisDivisionsScaleTiltAngle))
			double cosCop = Math.cos(Math.toRadians(coriolisIndexCoriolisDivisionsScaleTiltAngle))
			double computedTilt = -(abs*cosCop*coriolisGain)
			double computedPan = -(abs*sinCop*coriolisGain)
			if(tiltAngle>0){
				coriolisIndex++;
				coriolisIndex=(coriolisIndex>=coriolisDivisions?0:coriolisIndex)
			}else{
				coriolisIndex--;
				coriolisIndex=(coriolisIndex<0?coriolisDivisions-1:coriolisIndex)
			}
	
			double[] vect =tail.getCurrentJointSpaceVector()
			vect[0]=computedTilt
			vect[1]=computedPan
			
			tail.setDesiredJointSpaceVector(vect, 0)
		}
	}

	// Put the feet back into the neutral pose
	void doFinishingMove() {
		for(DHParameterKinematics leg:source.getLegs()) {
			ArrayList<TransformNR> feetTipsAll=legTipMap.get(leg)
			leg.setDesiredTaskSpaceTransform(feetTipsAll.get((int)0), 0)
		}
	}
	/**
	 * Determine if the leg is A group or B group
	 * @param leg the leg to check
	 * @return true if the leg is in group B, false otherwise
	 */
	boolean isAgroup(DHParameterKinematics leg) {
		// Check the limbs root for its location to determine corners
		TransformNR  legRoot= leg.getRobotToFiducialTransform()
		if(legRoot.getX()<=0&&legRoot.getY()<0 ){
			return true
		}else
		if(legRoot.getX()>=0&&legRoot.getY()>=0 ){
			return true
		}
		return false
	}
	/**
	 * Determine if the leg is A group or B group
	 * @param leg the leg to check
	 * @return true if the leg is in group B, false otherwise
	 */
	boolean isBgroup(DHParameterKinematics leg) {
		// Check the limbs root for its location to determine corners
		TransformNR  legRoot= leg.getRobotToFiducialTransform()
		if(legRoot.getX()>0&&legRoot.getY()<0 ){
			return true
		}else
		if(legRoot.getX()<0&&legRoot.getY()>=0 ){
			return true
		}
		return false
	}


	void doStep(){
		// for each leg in the cat
		for(DHParameterKinematics leg:source.getLegs()) {
			ArrayList<TransformNR> feetTipsAll=legTipMap.get(leg)
			def index =0
			// A group follows the index
			if(isAgroup( leg)) {
				index=pontsIndex
			}
			// B group is offset by half of the cycle.
			if(isBgroup( leg)) {
				index=pontsIndex+numberOfInterpolatedPointsInALoop/2
			}
			
			if(index>=numberOfInterpolatedPointsInALoop)
				index -=numberOfInterpolatedPointsInALoop
			//println "Leg "+leg.getScriptingName()+" index "+index
			// Get the tip location for this leg at the given index
			def newTip=feetTipsAll.get((int)index)
			// Check if the leg can achive the tip location and set
			if(leg.checkTaskSpaceTransform(newTip))
				leg.setDesiredTaskSpaceTransform(newTip, 0)
			else
				println "Leg "+leg.getScriptingName()+" failed in body controller"
		}
	}
	void setupCycle(){
		def maximumLoopTIme=cycleTime/1000.0
		if(seconds ==0)
			seconds=maximumLoopTIme
		while(seconds<maximumLoopTIme) {
			//println "Loop too fast, foot cycle took "+maximumLoopTIme+", targeted: "+seconds
			newPose=newPose.times(newPose)
			seconds=seconds+seconds
		}
		//rescale transform for one step cycle
		def scale = maximumLoopTIme/seconds
		if(scale>1.000001||scale<0.999999) {
			//println "Transform Scaled by "+scale
			seconds=maximumLoopTIme;
			newPose=newPose.scale(scale)
		}


		for(int i=1;i<100;i++) {
			try {
				clearLegTips()
				// generate the tip trajectories for all legs at all interpolated points
				def scaletmp = 1.0/((double)i)
				legTipMap =generateLegPlan(newPose.inverse().scale(scaletmp),numberOfInterpolationPoints,stepOverHeight,source)
				// if plan check passes, store the number of steps and move on
				break;
			}	catch(RuntimeException ex) {
				// the incoming transform is not possible, scale it into an integer number of steps
			}
		}
		//println "Cycle setup, will take "+newPose+" in "+seconds
	}

	void clearLegTips() {
		if(legTipMap!=null) {
			for(DHParameterKinematics K:legTipMap.keySet()) {
				legTipMap.get(K).clear()
			}
			legTipMap.clear()
			legTipMap=null;
		}
	}
	boolean connect() {
		println "Connecting Body Controller"
		availible=true;
		if(bodyLoop==null) {
			// Create a thread to run the body controller
			bodyLoop=new Thread({
				try {
					// initialize real-time initial conditions
					long start = System.currentTimeMillis();
					long index=0;
					// run body controller until disconnected
					while(availible) {
						// update the gait generation
						loop();
						// update the dynamics controller
						runDynamics();
						// compute the real-time condition
						long elapsed =  System.currentTimeMillis()-(start +(numMsOfLoop*index) )
						def numMsOfLoopElapsed = numMsOfLoop-elapsed
						// check for real-time overrun
						if(numMsOfLoopElapsed<=16) {
							println "Real time in Body Controller broken! Loop took:"+elapsed+" sleep time "+numMsOfLoopElapsed
							// this controller must run slower than the UI thread
							Thread.sleep(16);
						}else {
							// sleep for the computed amount of time to keep the start of loop consistant
							Thread.sleep(numMsOfLoopElapsed);
						}
						// update the real-time index
						index++
					}
				}catch(Throwable t) {
					BowlerStudio.printStackTrace(t)
				}
				// cleanup internal variables
				source=null;
				newPose=null;
				clearLegTips()
				println "Body Controller Thread exited ok"
				if(availible)
					disconnect();
				if(lastSource!=null) {
					if(lastSource.isAvailable()) {
						// undo the display
						lastSource.setGlobalToFiducialTransform(new TransformNR())
						lastSource.getImu().clearhardwareListeners()
					}
				}
			})
			bodyLoop.start();
		}
	}
	void disconnect() {
		println "Disconnecting Body Controller"
		availible=false;

	}

	/**
	 * Make a walking cycle as defined by the motions of the body
	 * @param transform the transform through which we want the body to move
	 * @param stepover the height above the ground plane for the foot to step over
	 * @info the returned array must have an even number of entries, and the same number of transforms moving forward as moving backward.
	 */
	ArrayList<TransformNR> makeCycle(TransformNR transform, def setepover){
		// assume the foot starts at home
		return [
			transform.scale(unitScale),
			// first move backwards 1/6 of the transform
			transform.scale(unitScale),
			// first move backwards 1/6 of the transform
			transform.scale(unitScale),
			// first move backwards 1/6 of the transform
			transform.scale(unitScale).translateZ(setepover),
			// keep moving backwards and move up
			transform.inverse().scale(unitScale*2),
			// move foot forward, against the body movement direction
			transform.inverse().scale(unitScale*2),
			//
			transform.inverse().scale(unitScale*2),
			//
			transform.inverse().scale(unitScale*2),
			//
			transform.scale(unitScale).translateZ(-setepover),
			// move foot forward, against the body movement direction again to ensure an identical number of up and down move
			transform.scale(unitScale),
			// moving back 1/6 of the transform
			transform.scale(unitScale),
			// moving back 1/6 of the transform
			transform.scale(unitScale),// moving back 1/6 of the transform
		]
	}
	/**
	 * computeOneFootPoseChange
	 *
	 * @param cat the Mobile base instance of the body
	 * @param leg the specific leg we are working with
	 * @param T_tg the pose of the limb to begin with in global space
	 * @param T_deltBody the new motion we want the body to move through
	 * @return the new computed tip in global space
	 */
	TransformNR computeOneFootPoseChange(MobileBase cat,DHParameterKinematics leg, TransformNR T_tg,TransformNR T_deltBody) {
		// get the static transformations
		TransformNR T_f_l = leg.getRobotToFiducialTransform()
		TransformNR T_g_f = cat.getFiducialToGlobalTransform()
		// convert the previous tip location to limb space
		TransformNR T_tl = leg.inverseOffset(T_tg)
		// compute the new body pose based on the incoming delta
		TransformNR T_gdelt = T_g_f.times(T_deltBody)
		// compute the the global tip based on the updated body pose
		TransformNR T_newtip = T_gdelt.times(T_f_l).times(T_tl)
		return T_newtip
	}
	/**
	 * computeFootCorners
	 * @param cat  the Mobile base instance of the body
	 * @param leg the specific leg we are working with
	 * @param cycle the set of poses to move the body through to produce the foot trajectory
	 * @return a set of tip locations that correspond to the tip of the foot moving through the body pose cycle
	 */
	ArrayList<TransformNR> computeFootCorners(MobileBase cat,DHParameterKinematics leg,ArrayList<TransformNR> cycle) {
		// Compute the "home" location for the feet, where do they start the cycle
		TransformNR T_tg = cat.calcHome(leg)
		ArrayList<TransformNR> tips = []
		// The previous tip is initialized to the "home" for the given limb
		def lastTip = T_tg
		// for each body delta in the trajectory plan
		for(int i=0;i<cycle.size();i++) {
			// Compute the new global space foot location
			// assume the previous tip location is the starting point (lastTip)
			def newTip = computeOneFootPoseChange(cat,leg,lastTip,   cycle.get(i)     )
			// Check to make sure the leg can achieve the new targeted pose
			if(!leg.checkTaskSpaceTransform(newTip)) {
				// kick this up stream and force the cats trajectory planner to deal with this error
				throw new RuntimeException("Not possible body move !")
			}
			// if the tip is achievable, store it
			tips.add(newTip)
			// save this tip and the lastTip for next iteration of the loop
			lastTip=newTip
		}
		return tips
	}
	/**
	 * computeTipsWithInterpolation
	 *
	 *  compute all of the intermediate points for a list of tip corners
	 *
	 * @param feetTips the corners of the legs tips
	 * @param numberOfInterpolationPoints how many intermediate points to add to the corners
	 * @return a larger array of the tip trajectory with interpolation between corners
	 */
	ArrayList<TransformNR> computeTipsWithInterpolation(ArrayList<TransformNR> feetTips, double numberOfInterpolationPoints){
		// make a new larger list for all the interpolated points
		ArrayList<TransformNR> tipsAll = []
		// grab the last index in the corners as the "previous" tip starting point
		TransformNR previous=feetTips.get(feetTips.size()-1)
		// for each corner in the trajectory plan
		for(int i=0;i<feetTips.size();i++) {
			// get the tip location target for this cycle
			TransformNR newTip=feetTips.get(i)
			// compute the delta from previous tip and this one
			TransformNR delta =previous.inverse().times(newTip)
			double numInterpPoints= numberOfInterpolationPoints+1.0
			double increment =(1.0/numInterpPoints)
			// for each interpolated point
			for(double j=increment;j<=1;j+=(1/numInterpPoints)) {
				// scale the delta from 0 to 1 (j)
				// and apply it to the previous transform
				def interm =previous.times(delta.scale(j))
				// store the intermediate point
				tipsAll.add(interm)
			}
			// save the current tip as the new tip for next loop
			previous=newTip
		}
		return tipsAll
	}
	/**
	 * generateLegPlan
	 *
	 * generate the tip trajectories for all legs at all interpolated points
	 *
	 * @param T_deltBody a single transform that the body should move
	 * @param numberOfInterpolationPoints the number of points that the interpolation step uses
	 * @param stepoverHeight the height of the foot above the home point to step over
	 * @param cat   the Mobile base instance of the body
	 * @return a hashmap of leg to tip trajectory list
	 */
	HashMap<DHParameterKinematics,ArrayList<TransformNR>> generateLegPlan(
			TransformNR T_deltBody,
			int numberOfInterpolationPoints,
			double stepoverHeight,
			MobileBase cat){
		// Generate the 12 Delta body transforms to take the foot through a cycle
		def cycle = makeCycle(T_deltBody, stepoverHeight)
		// make a hashmap to store the leg to point array data
		HashMap<DHParameterKinematics,ArrayList<TransformNR>> legTipMap =new HashMap<>()
		// for each leg in the cat
		for(DHParameterKinematics leg:cat.getLegs()) {
			// compute all the foot corners for a given trajectory plan
			// NOTE this will throw an exception if one of the points isn't achievable by the leg
			ArrayList<TransformNR> feetCorners =computeFootCorners( cat, leg,cycle)
			// use the leg tip corners to generate interpolation points in between the corners
			ArrayList<TransformNR> feetTipsAll = computeTipsWithInterpolation(feetCorners,numberOfInterpolationPoints)
			// store the fully interpolated trajectory plan
			legTipMap.put(leg,feetTipsAll)
		}
		return legTipMap
	}


}















