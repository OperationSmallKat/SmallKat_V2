import com.neuronrobotics.bowlerstudio.creature.ICadGenerator;
import com.neuronrobotics.bowlerstudio.creature.CreatureLab;
import org.apache.commons.io.IOUtils;
import com.neuronrobotics.bowlerstudio.vitamins.*;
import java.nio.file.Paths;
import eu.mihosoft.vrl.v3d.FileUtil;
import eu.mihosoft.vrl.v3d.Transform;
import javafx.scene.transform.Affine;
import com.neuronrobotics.bowlerstudio.physics.TransformFactory;
println "Loading STL file"
// Load an STL file from a git repo
// Loading a local file also works here

return new ICadGenerator(){
	
	private CSG moveDHValues(CSG incoming,DHLink dh ){
		TransformNR step = new TransformNR(dh.DhStep(0)).inverse()
		Transform move = TransformFactory.nrToCSG(step)
		return incoming.transformed(move)
		
	}

	@Override 
	public ArrayList<CSG> generateCad(DHParameterKinematics d, int linkIndex) {
		ArrayList<CSG> allCad=new ArrayList<>();
		String limbName = d.getScriptingName()
		File legFile = null
		boolean mirror=true
		if(limbName.contentEquals("DefaultLeg3")||limbName.contentEquals("DefaultLeg4")){
			println "Mirror leg parts"
			mirror=false
		}
		if(limbName.contentEquals("Tail")){
			if(linkIndex ==0){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Elbow Joint.STL");
	
			}
			if(linkIndex ==1){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Tail.STL");
			}
	
			if(linkIndex ==2)
				return allCad;
		}else if(limbName.contentEquals("Head")){
			if(linkIndex ==0){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Elbow Joint.STL");

			}
			if(linkIndex ==1){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Head.STL");
			}
	
			if(linkIndex ==2)
				return allCad;
		}else{
			if(linkIndex ==0){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Elbow Joint.STL");
	
			}
			if(linkIndex ==1){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Midleg.STL");
	
			}
	
			if(linkIndex ==2){
				legFile = ScriptingEngine.fileFromGit(
				"https://github.com/keionbis/SmallKat.git",
				"STLs/Lower Leg.STL");
	
			}
		}
		


		ArrayList<DHLink> dhLinks = d.getChain().getLinks()
		DHLink dh = dhLinks.get(linkIndex)
		// Hardware to engineering units configuration
		LinkConfiguration conf = d.getLinkConfiguration(linkIndex);
		// Engineering units to kinematics link (limits and hardware type abstraction)
		AbstractLink abstractLink = d.getAbstractLink(linkIndex);// Transform used by the UI to render the location of the object
		// Transform used by the UI to render the location of the object
		Affine manipulator = dh.getListener();

		
		// Load the .CSG from the disk and cache it in memory
		CSG body  = Vitamins.get(legFile)
		if(linkIndex ==0){
			body=moveDHValues(body,dh)
				.movex(-4.5)
				.movez(-20)				
				.movey(mirror?5:15.5)
				.rotx(mirror?180:0)
			if(limbName.contentEquals("Head")||limbName.contentEquals("Tail")){
				body=body
					.movez(-11.5)
			}	
				
		}
		if(linkIndex ==1){
			body=moveDHValues(body,dh)
				.movey(-9)
				.movex(-9)
				.movez(-11)
				.rotx(!mirror?180:0)
			if(limbName.contentEquals("Head")){
				body=body
					.movex(35.5)
					.movez(-11.5)
			}
		}
		if(linkIndex ==2){
			body=moveDHValues(body.rotz(-90),dh)
				.movey(-8)
				.movex(-8.5)
				.movez(-20)
				.rotx(mirror?180:0)
		}
		body.setManipulator(manipulator);
	
		def parts = [body ] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setColor(javafx.scene.paint.Color.RED)
		}
		return parts;
		
	}
	@Override 
	public ArrayList<CSG> generateBody(MobileBase b ) {
		ArrayList<CSG> allCad=new ArrayList<>();

		File mainBodyFile = ScriptingEngine.fileFromGit(
			"https://github.com/keionbis/SmallKat.git",
			"STLs/BodyV2.STL");

		// Load the .CSG from the disk and cache it in memory
		CSG body  = Vitamins.get(mainBodyFile)
					.movex(-79)
					.movey(-38)
					.movez(57)

		body.setManipulator(b.getRootListener());
		body.setColor(javafx.scene.paint.Color.WHITE)
		def parts = [body ] as ArrayList<CSG>
		for(int i=0;i<parts.size();i++){
			parts.get(i).setColor(javafx.scene.paint.Color.GRAY)
		}
		return parts;
	}
};