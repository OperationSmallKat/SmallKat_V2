
def cat =ScriptingEngine.gitScriptRun("https://github.com/keionbis/SmallKat.git", "loadRobot.groovy", null);

def gameController = ScriptingEngine.gitScriptRun(
            "https://gist.github.com/e26c0d8ef7d5283ef44fb22441a603b8.git", // git location of the library
            "LoadGameController.groovy" , // file to load
            // Parameters passed to the function
            ["GameController*"]
            )
         
if(gameController==null){

	return 
}

byte [] data = gameController.getData() 
double toSeconds=0.03//100 ms for each increment

while (!Thread.interrupted()){
	Thread.sleep((long)(toSeconds*1000))
	double xdata = data[2]
	double rzdata = data[1]
	if(xdata<0)
		xdata+=256
	if(rzdata<0)
		rzdata+=256
	double scale = 1.0
	double displacement = scale*xdata/255.0-scale/2
	double rot =scale*rzdata/255.0-scale/2
	println "displacement "+displacement+" rot "+rot
	TransformNR move = new TransformNR(displacement,0,0,new RotationNR(0,rot,0))
	cat.DriveArc(move, toSeconds);
}
