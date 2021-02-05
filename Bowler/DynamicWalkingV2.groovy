import com.neuronrobotics.sdk.addons.kinematics.DHParameterKinematics
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.Log


MobileBase cat = DeviceManager.getSpecificDevice("SmallKatGenerated")

/**
 * Make a walking cycle as defined by the motions of the body
 * @param transform the transform through which we want the body to move
 * @param stepover the height above the ground plane for the foot to step over
 * @info the returned array must have an even number of entries, and the same number of transforms moving forward as moving backward.
 */
ArrayList<TransformNR> makeCycle(TransformNR transform, def setepover){
	double unitScale =1.0/6.0
	// assume the foot starts at home
	return [
		transform.scale(unitScale),// first move backwards 1/6 of the transform
		transform.scale(unitScale),// first move backwards 1/6 of the transform
		transform.scale(unitScale),// first move backwards 1/6 of the transform
		transform.scale(unitScale).translateZ(setepover),// keep moving backwards and move up
		transform.inverse().scale(unitScale*2),// move foot forward, against the body movement direction
		transform.inverse().scale(unitScale*2),//
		transform.inverse().scale(unitScale*2),//
		transform.inverse().scale(unitScale*2),//
		transform.scale(unitScale).translateZ(-setepover),// move foot forward, against the body movement direction again to ensure an identical number of up and down move
		transform.scale(unitScale),// moving back 1/6 of the transform
		transform.scale(unitScale),// moving back 1/6 of the transform
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
						.translateZ(-12.5)// move the starting point down
						.translateX(10); // move the starting point forward
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
/**
 * 
 * @param cat  the Mobile base instance of the body
 * @param T_deltBody  a single transform that the body should move
 * @param seconds how long the translation of the body should take
 */
public void DriveArc(MobileBase cat, TransformNR T_deltBody, double seconds) {
	double maximumLoopTIme = 0.15
	int numberOfInterpolationPoints=10
	double stepOverHeight = 15
	if(seconds ==0)
		seconds=maximumLoopTIme
	// check for the minimum time boundary and recoursivly increase the transfom 
	// and time together to preserve the targeted velocity
	if(seconds<maximumLoopTIme) {
		println "Loop too fast, foot cycle must take at least "+maximumLoopTIme+", targeted: "+seconds
		DriveArc( cat,  T_deltBody.times(T_deltBody),  seconds+seconds)
		return;
	}

	HashMap<DHParameterKinematics,ArrayList<TransformNR>> legTipMap;
	int numSteps=1;
	for(int i=1;i<100;i++) {
		try {
			// generate the tip trajectories for all legs at all interpolated points
			legTipMap =generateLegPlan(T_deltBody.inverse().scale(1.0/((double)i)),numberOfInterpolationPoints,stepOverHeight,cat)
			// if plan check passes, store the number of steps and move on
			numSteps=i;
			break;
		}	catch(RuntimeException ex) {
			// the incoming transform is not possible, scale it into an integer number of steps
		}
	}
	// Check for too-fast condition and fail out if too fast
	if((maximumLoopTIme*(double)numSteps)>(seconds)) {
		println "Commanded speed Too fast can not do "+T_deltBody+" in "+seconds+" seconds"
		return
	}
	
	println "Taking "+numSteps+" step(s)"
	// read how many points need to be added
	int points = legTipMap.get(cat.getLegs().get(0)).size()
	def seconds1000PointsNumSteps = (long)((seconds*1000)/((double)points)/((double)numSteps))
	// for each step
	for(int x=0;x<numSteps;x++) {
		// for every point in the trajectory
		for(int i=0;i<points;i++) {
			// for each leg in the cat
			for(DHParameterKinematics leg:cat.getLegs()) {
				ArrayList<TransformNR> feetTipsAll=legTipMap.get(leg)
				// A group follows the index
				def index = i;
				// B group is offset by half of the cycle. 
				if(isBgroup( leg)) {
						index+=points/2
						if(index>=points)
							index -=points
				}
				// Get the tip location for this leg at the given index
				def newTip=feetTipsAll.get((int)index)
				// Check if the leg can achive the tip location and set
				if(leg.checkTaskSpaceTransform(newTip))
					leg.setDesiredTaskSpaceTransform(newTip, 0)
			}
			
			println "Sleep for "+seconds1000PointsNumSteps
			if(seconds1000PointsNumSteps<20)
				seconds1000PointsNumSteps=20
			Thread.sleep(seconds1000PointsNumSteps)
		}
	}
	//set limbs all to home locations
	for(DHParameterKinematics leg:cat.getLegs()) {
		ArrayList<TransformNR> feetTipsAll=legTipMap.get(leg)
		leg.setDesiredTaskSpaceTransform(feetTipsAll.get((int)0), 0)
	}
}

TransformNR T_deltBody = new TransformNR(0, 0, 0, new RotationNR(0,90,0))
DriveArc(cat,T_deltBody,2)










