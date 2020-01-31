@GrabResolver(name='nr', root='https://oss.sonatype.org/content/repositories/staging/')
@Grab(group='com.neuronrobotics', module='SimplePacketComsJava', version='0.12.0')

import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.phy.*;

import com.neuronrobotics.bowlerstudio.creature.MobileBaseLoader
import com.neuronrobotics.sdk.addons.kinematics.AbstractRotoryLink
import com.neuronrobotics.sdk.addons.kinematics.LinkConfiguration
import com.neuronrobotics.sdk.addons.kinematics.LinkFactory
import com.neuronrobotics.sdk.addons.kinematics.MobileBase
import com.neuronrobotics.sdk.addons.kinematics.imu.*;
import com.neuronrobotics.sdk.common.DeviceManager
import com.neuronrobotics.sdk.common.NonBowlerDevice

import edu.wpi.SimplePacketComs.BytePacketType;
import edu.wpi.SimplePacketComs.FloatPacketType;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.phy.UDPSimplePacketComs;
import edu.wpi.SimplePacketComs.device.gameController.*;
import edu.wpi.SimplePacketComs.device.*
if(args == null)
	args = ["https://github.com/OperationSmallKat/greycat.git",
		"MediumKat.xml","GameController_22","hidDevice"]
def hidDeviceName = "hidDevice"
def obotDevicename ="MediumKat"
if(args.size()>3)
	hidDeviceName=args[3]
if(args.size()>4)
	obotDevicename=args[4]

try{

def gc= DeviceManager.getSpecificDevice(args[2],{
	return  ScriptingEngine.gitScriptRun(
            "https://gist.github.com/e26c0d8ef7d5283ef44fb22441a603b8.git", // git location of the library
            "LoadGameController.groovy" , // file to load
            // Parameters passed to the function
            [args[2]]
            )
	})
}catch (Throwable t){}

MobileBase cat =DeviceManager.getSpecificDevice( obotDevicename,{
	//If the device does not exist, prompt for the connection
	
	MobileBase m = MobileBaseLoader.fromGit(
		args[0],
		args[1]
		)
	def dev = DeviceManager.getSpecificDevice( hidDeviceName)	
	dev.simple.addEvent(1804, {
		 double[] imuDataValues = dev.simple.getImuData()
		 m.getImu()
		 .setHardwareState(
		 		new IMUUpdate(
		 			-imuDataValues[9],	-imuDataValues[11],	-imuDataValues[10],
					imuDataValues[3],//Double rotxAcceleration,
					imuDataValues[4],//Double rotyAcceleration,
					imuDataValues[5],//Double rotzAcceleration 
			))
		 
		 
	});
	
	if(m==null)
		throw new RuntimeException("Arm failed to assemble itself")
	println "Connecting new device robot arm "+m
	return m
})

return cat
