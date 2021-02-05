package Bowler;

import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.IDriveEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.Log
enum CycleState {
	waiting,
	cycleStart,
	stepThroughPoints,
	checkForContinue,
	cycleFinish
}
class BodyController{
	int numberOfInterpolationPoints=1
	double stepOverHeight = 15

	double zoffsetOfFeetHome = -12.5
	double xOffsetOfFeetHome = 10
	double numPointsInLoop =12.0
	double unitScale =1.0/(numPointsInLoop/2.0)
	Thread bodyLoop = null;
	boolean availible=true;
	int numMsOfLoop = 20;
	double cycleTime = numMsOfLoop*numPointsInLoop*numberOfInterpolationPoints
	int numberOfInterpolatedPointsInALoop = numPointsInLoop*numberOfInterpolationPoints
	MobileBase source=null;
	TransformNR newPose=null;
	double seconds=0;
	long timeOfMostRecentCommand=0;
	boolean cycleStarted=false;

	def state=CycleState.waiting;
	int pontsIndex=0; 
	HashMap<DHParameterKinematics,ArrayList<TransformNR>> legTipMap=null;
	
	private void loop() {
		double timeElapsedSinceLastCommand = ((double)(System.currentTimeMillis()-timeOfMostRecentCommand))/1000.0
		switch(state) {
			case CycleState.waiting:
				if(source!=null) {
					state=CycleState.cycleStart;
					println "Walk cycle starting "+cycleTime;
				}else					
					break;
			case CycleState.cycleStart:
				pontsIndex=0;
				setupCycle()
				println "Starting steps"
				state=CycleState.stepThroughPoints
				//no break
			case CycleState.stepThroughPoints:
				if(pontsIndex<numberOfInterpolatedPointsInALoop) {
					println "Interpolation point "+pontsIndex+" time elapsed "+timeElapsedSinceLastCommand
					doStep()
					pontsIndex++;
				}else {					
					state=CycleState.checkForContinue
				}
				break;
			case CycleState.checkForContinue:
				if((timeElapsedSinceLastCommand-(cycleTime/1000.0))<seconds) {
					state=CycleState.cycleStart
					println "Cycle not finished, stepping again"
					break;
				}else {
					state=CycleState.cycleFinish
				}
				break;
			case CycleState.cycleFinish:
				 doFinishingMove()
				state=CycleState.waiting
				println "Finising move"
				source=null
				clearLegTips()
				break;
		}
	}
	
	void doFinishingMove() {
		for(DHParameterKinematics leg:source.getLegs()) {
			ArrayList<TransformNR> feetTipsAll=legTipMap.get(leg)
			leg.setDesiredTaskSpaceTransform(feetTipsAll.get((int)0), 0)
		}
	}
	void doStep(){
		// for each leg in the cat
		for(DHParameterKinematics leg:source.getLegs()) {
			ArrayList<TransformNR> feetTipsAll=legTipMap.get(leg)
			// A group follows the index
			def index = pontsIndex;
			// B group is offset by half of the cycle.
			if(isBgroup( leg)) {
				index+=numberOfInterpolatedPointsInALoop/2
				if(index>=numberOfInterpolatedPointsInALoop)
					index -=numberOfInterpolatedPointsInALoop
			}
			// Get the tip location for this leg at the given index
			def newTip=feetTipsAll.get((int)index)
			// Check if the leg can achive the tip location and set
			if(leg.checkTaskSpaceTransform(newTip))
				leg.setDesiredTaskSpaceTransform(newTip, 0)
		}
	}
	void setupCycle(){
		def maximumLoopTIme=cycleTime/1000.0
		if(seconds ==0)
			seconds=maximumLoopTIme
		while(seconds<maximumLoopTIme) {
			println "Loop too fast, foot cycle must take at least "+maximumLoopTIme+", targeted: "+seconds
			newPose=newPose.times(newPose)
			seconds=seconds+seconds
		}
		int numSteps=1;
		for(int i=1;i<100;i++) {
			try {
				clearLegTips()
				// generate the tip trajectories for all legs at all interpolated points
				legTipMap =generateLegPlan(newPose.inverse().scale(1.0/((double)i)),numberOfInterpolationPoints,stepOverHeight,source)
				// if plan check passes, store the number of steps and move on
				numSteps=i;
				break;
			}	catch(RuntimeException ex) {
				// the incoming transform is not possible, scale it into an integer number of steps
			}
		}
		println "Cycle setup, will take "+numSteps+" steps"
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
			bodyLoop=new Thread({
				try {
					while(availible) {
						long start = System.currentTimeMillis();
						loop();
						long elapsed =  System.currentTimeMillis()-start
						def numMsOfLoopElapsed = numMsOfLoop-elapsed
						if(numMsOfLoopElapsed<16) {
							println "Real time in Body Controller broken! Loop took:"+elapsed+" sleep time "+numMsOfLoopElapsed
							Thread.sleep(16);
						}else
							Thread.sleep(numMsOfLoopElapsed);
					}
					source=null;
					newPose=null;
					clearLegTips()
				}catch(Throwable t) {
					BowlerStudio.printStackTrace(t)
					disconnect();
				}
				println "Body Controller Thread exited ok"
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
		TransformNR T_tg = leg.calcHome()
				.translateZ(zoffsetOfFeetHome)// move the starting point down
				.translateX(xOffsetOfFeetHome); // move the starting point forward
		// store the 12 global space tips of the given limb
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
			// for each interpolated point
			for(double j=0;j<=1;j+=(1/numberOfInterpolationPoints)) {
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
		if(legRoot.getX()<0&&legRoot.getY()>0 ){
			return true
		}
		return false
	}


}
MobileBase cat = DeviceManager.getSpecificDevice("SmallKatGenerated")



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
			public void DriveArc(MobileBase source,TransformNR newPose,double seconds) {
				def con = DeviceManager.getSpecificDevice("BodyController-"+cat.getScriptingName(),{
					BodyController bc= new BodyController()
					bc.connect();
					return bc;
				})
				con.source=source
				con.newPose=newPose
				con.seconds=seconds
				con.timeOfMostRecentCommand=System.currentTimeMillis()
			}
		}

TransformNR T_deltBody = new TransformNR(0, 0, 0, new RotationNR(0,0.9,0))
for(int i=0;i<100;i++) {
	engine.DriveArc(cat,T_deltBody,0.02)
	Thread.sleep(20)
}










