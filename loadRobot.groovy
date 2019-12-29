@GrabResolver(name='nr', root='https://oss.sonatype.org/content/repositories/staging/')
@Grab(group='com.neuronrobotics', module='SimplePacketComsJava', version='0.12.0')
@Grab(group='com.neuronrobotics', module='SimplePacketComsJava-HID', version='0.13.0')
@Grab(group='org.hid4java', module='hid4java', version='0.5.0')

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
public class SimpleServoHID extends HIDSimplePacketComs {
	private PacketType servos = new edu.wpi.SimplePacketComs.BytePacketType(1962, 64);
	private PacketType imuData = new edu.wpi.SimplePacketComs.FloatPacketType(1804, 64);
	private final double[] status = new double[12];
	private final byte[] data = new byte[16];
	
	public SimpleServoHID(int vidIn, int pidIn) {
		super(vidIn, pidIn);
		servos.waitToSendMode();
		addPollingPacket(servos);
		addPollingPacket(imuData);
		addEvent(1962, {
			writeBytes(1962, data);
		});
		addEvent(1804, {
			readFloats(1804,status);
		});
	}
	public double[] getImuData() {
		return status;
	}
	public byte[] getData() {
		return data;
	}
	public byte[] getDataUp() {
		return servos.getUpstream();
	}
}

public class SimpleServoUDPServo extends UDPSimplePacketComs {
	private PacketType servos = new edu.wpi.SimplePacketComs.BytePacketType(1962, 64);
	private final byte[] data = new byte[16];
	public SimpleServoUDPServo(def address) {
		super(address);
		servos.waitToSendMode();
		addPollingPacket(servos);
		addEvent(1962, {
			writeBytes(1962, data);
		});
	}
	public byte[] getDataUp() {
		//return servos.getUpstream();
		return data;
	}
	public byte[] getData() {
		return data;
	}
}

public class SimpleServoUDPImu extends UDPSimplePacketComs {
	private PacketType imuData = new edu.wpi.SimplePacketComs.FloatPacketType(1804, 64);
	private final double[] status = new double[12];
	
	public SimpleServoUDPImu(def address) {
		super(address);
		addPollingPacket(imuData);
		addEvent(1804, {
			readFloats(1804,status);
		});
	}
	public double[] getImuData() {
		return status;
	}
}

public class HIDSimpleComsDevice extends NonBowlerDevice{
	def simple;
	def simpleServo;
	public HIDSimpleComsDevice(def simp, def servo){
		simple = simp
		simpleServo=servo
		setScriptingName("hidbowler")
	}
	@Override
	public  void disconnectDeviceImp(){		
		simple.disconnect()
		simpleServo.disconnect()
		println "HID device Termination signel shutdown"
	}
	
	@Override
	public  boolean connectDeviceImp(){
		simple.connect()
		simpleServo.connect()
	}
	void setValue(int i,int position){
		simpleServo.getData()[i]=(byte)position;
		simpleServo.servos.pollingMode();
	}
	int getValue(int i){
		if(simpleServo.getDataUp()[i]>0)
			return simpleServo.getDataUp()[i]
		return ((int)simpleServo.getDataUp()[i])+256
	}
	public float[] getImuData() {
		return simple.getImuData();
	}
	@Override
	public  ArrayList<String>  getNamespacesImp(){
		// no namespaces on dummy
		return [];
	}
	
	
}

public class HIDRotoryLink extends AbstractRotoryLink{
	HIDSimpleComsDevice device;
	int index =0;
	int lastPushedVal = 0;
	private static final Integer command =1962
	/**
	 * Instantiates a new HID rotory link.
	 *
	 * @param c the c
	 * @param conf the conf
	 */
	public HIDRotoryLink(HIDSimpleComsDevice c,LinkConfiguration conf,String devName) {
		super(conf);
		index = conf.getHardwareIndex()
		device=c
		if(device ==null)
			throw new RuntimeException("Device can not be null")
		conf.setDeviceScriptingName(devName)	
		c.simpleServo.addEvent(command,{
			int val= getCurrentPosition();
			if(lastPushedVal!=val){
				//println "Fire Link Listner "+index+" value "+getCurrentPosition()
				try{
				fireLinkListener(val);
				}catch(Exception e){}
				lastPushedVal=val
			}else{
				//println index+" value same "+getCurrentPosition()
			}
			
		})
		
	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#cacheTargetValueDevice()
	 */
	@Override
	public void cacheTargetValueDevice() {
		device.setValue(index,(int)getTargetValue())
	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#flush(double)
	 */
	@Override
	public void flushDevice(double time) {
		// auto flushing
	}
	
	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#flushAll(double)
	 */
	@Override
	public void flushAllDevice(double time) {
		// auto flushing
	}

	/* (non-Javadoc)
	 * @see com.neuronrobotics.sdk.addons.kinematics.AbstractLink#getCurrentPosition()
	 */
	@Override
	public double getCurrentPosition() {
		return (double)device.getValue(index);
	}

}
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
def dev = DeviceManager.getSpecificDevice( hidDeviceName,{
	//If the device does not exist, prompt for the connection
	def simp = null;
	def srv = null
	
	HashSet<InetAddress> addresses = UDPSimplePacketComs.getAllAddresses(hidDeviceName);
	
	if (addresses.size() < 1) {
			simp = new SimpleServoHID(0x16C0 ,0x0486) 
			srv=simp
	}else{
		println "Servo Servers at "+addresses
		simp = new SimpleServoUDPImu(addresses.toArray()[0])
		simp.setReadTimeout(20);
		srv = new SimpleServoUDPServo(addresses.toArray()[0])
		srv.setReadTimeout(20);
		
	}
	HIDSimpleComsDevice d = new HIDSimpleComsDevice(simp,srv)
	d.connect(); // Connect to it.
	if(simp.isVirtual()){
		println "\n\n\nDevice is in virtual mode!\n\n\n"
	}

	LinkFactory.addLinkProvider("hidfast",{LinkConfiguration conf->
				//println "Loading link "
				return new HIDRotoryLink(d,conf,hidDeviceName)
		}
	)
	println "Connecting new device: "+d
	return d
})

MobileBase cat =DeviceManager.getSpecificDevice( obotDevicename,{
	//If the device does not exist, prompt for the connection
	
	MobileBase m = MobileBaseLoader.fromGit(
		args[0],
		args[1]
		)
		
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
