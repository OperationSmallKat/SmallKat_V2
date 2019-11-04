import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.math.RotationNR
import com.neuronrobotics.sdk.addons.kinematics.math.TransformNR

MobileBase cat =ScriptingEngine.gitScriptRun(	"https://github.com/OperationSmallKat/SmallKat_V2.git", 
								"loadRobot.groovy", 
["https://github.com/OperationSmallKat/SmallKat_V2.git",
		"Bowler/MediumKat.xml"]);

def gameController = ScriptingEngine.gitScriptRun(
            "https://gist.github.com/e26c0d8ef7d5283ef44fb22441a603b8.git", // git location of the library
            "LoadGameController.groovy" , // file to load
            // Parameters passed to the function
            ["KatControl"]
            )
         
if(gameController==null){

	return 
}

byte [] data = gameController.getData() 
double toSeconds=0.03//100 ms for each increment

while (!Thread.interrupted()){
	Thread.sleep(30)
	gameController.getData(data)
	//println gameController
	double xdata = data[4]
	double rzdata = data[3]
	double rxdata = data[1]
	double rydata = data[2]
	
	if(xdata<0)
		xdata+=256
	if(rzdata<0)
		rzdata+=256
	if(rxdata<0)
		rxdata+=256
	if(rydata<0)
		rydata+=256
	double scale = 1.0
	double displacement = 15*(scale*xdata/255.0-scale/2)
	double rot =((scale*rzdata/255.0)-scale/2)*-2.5
	double rotx =((rxdata/255.0)-scale/2)*5
	double roty =((rydata/255.0)-scale/2)*-5
	if(Math.abs(displacement)<0.1 ){
		displacement=0
	}
	if( Math.abs(rot)<0.1){
		rot=0
	}
	try{
		if(Math.abs(rotx)>0.1 || Math.abs(roty)>0.1){
			TransformNR move = new TransformNR(displacement,0,0,new RotationNR(rotx,0,roty))
			cat.getWalkingDriveEngine().pose(move)
		}
		if(Math.abs(displacement)>0.1 || Math.abs(rot)>0.1){
			print "\r\ndisplacement "+displacement+" rot "+rot
			print " tilt "+rotx+" rot "+roty
			print " rawX "+xdata+" rawZ "+rzdata+" data "+data+"\r\n"
			TransformNR move = new TransformNR(displacement,0,0,new RotationNR(rotx,rot,roty))
			cat.DriveArc(move, toSeconds);
		}
	}
	catch(Throwable t){}
	
}
