@GrabResolver(name='nr', root='https://oss.sonatype.org/content/repositories/staging/')
@Grab(group='com.neuronrobotics', module='SimplePacketComsJava', version='0.9.0')
@Grab(group='com.neuronrobotics', module='SimplePacketComsJava-HID', version='0.1.0')
@Grab(group='org.hid4java', module='hid4java', version='0.5.0')

import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.phy.*;
import com.neuronrobotics.sdk.addons.kinematics.imu.*;
import edu.wpi.SimplePacketComs.BytePacketType;
import edu.wpi.SimplePacketComs.FloatPacketType;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.phy.UDPSimplePacketComs;
import edu.wpi.SimplePacketComs.device.gameController.*;
import edu.wpi.SimplePacketComs.device.*
if(args == null)
	args = ["https://github.com/OperationSmallKat/SmallKat_V2.git",
		"Bowler/MediumKat.xml","GameController_22"]

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
}

public class SimpleServoUDP extends UDPSimplePacketComs {
	private PacketType servos = new edu.wpi.SimplePacketComs.BytePacketType(1962, 64);
	private PacketType imuData = new edu.wpi.SimplePacketComs.FloatPacketType(1804, 64);
	private final double[] status = new double[12];
	private final byte[] data = new byte[16];
	
	public SimpleServoUDP(def address) {
		super(address);
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
}


public class HIDSimpleComsDevice extends NonBowlerDevice{
	def simple;
	
	public HIDSimpleComsDevice(def simp){
		simple = simp
		setScriptingName("hidbowler")
	}
	@Override
	public  void disconnectDeviceImp(){		
		simple.disconnect()
		println "HID device Termination signel shutdown"
	}
	
	@Override
	public  boolean connectDeviceImp(){
		simple.connect()

	}
	void setValue(int i,int position){
		simple.getData()[i]=(byte)position;
		simple.servos.pollingMode();
	}
	int getValue(int i){
		if(simple.getData()[i]>0)
			return simple.getData()[i]
		return ((int)simple.getData()[i])+256
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
	public HIDRotoryLink(HIDSimpleComsDevice c,LinkConfiguration conf) {
		super(conf);
		index = conf.getHardwareIndex()
		device=c
		if(device ==null)
			throw new RuntimeException("Device can not be null")
		c.simple.addEvent(command,{
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

public class GameControllerLocal extends UdpDevice{
	private PacketType gamestate = new BytePacketType(1970, 64);
	private  byte[] status = new byte[60];
	private  byte[] d = new byte[20];
	private  byte[] dataTmp = new byte[20];
	private final myLock = new Object()
	private GameControllerLocal(InetAddress add) throws Exception {
		super(add);
		setReadTimeout(200)
		addPollingPacket(gamestate);
		addEvent(gamestate.idOfCommand, {
			readBytes(gamestate.idOfCommand, dataTmp);
			writeBytes(gamestate.idOfCommand, getStatus());
			
			for(int i=1;i<20;i++){
				if(dataTmp[i]!=0){
					synchronized(myLock){
						for(int j=0;j<20;j++){
							d[j]=dataTmp[j];
						}
					}
					//println this
					return
				}
			}
			
			//println "Controller COMS FAULT "+this
			
		});
	}
	public static List<GameControllerLocal> getc(String n) throws Exception {
		HashSet<InetAddress> addresses = UDPSimplePacketComs.getAllAddresses(n);
		ArrayList<GameControllerLocal> robots = new ArrayList<>();
		if (addresses.size() < 1) {
			System.out.println("No GameControllers found named "+n);
			return robots;
		}
		for (InetAddress add : addresses) {
			System.out.println("Got " + add.getHostAddress());
			GameControllerLocal e = new GameControllerLocal(add);
			robots.add(e);
		}
		return robots;
	}

	public byte[] getStatus() {
		return status;
	}
	public void getData(byte[] back) {
		synchronized(myLock){
			for(int j=0;j<20;j++)
				back[j]=d[j];
		}
	}
	
	public byte[] getData() {
		byte[] back = new byte[20];
		getData( back)
		return back
	}
	
	public byte getControllerIndex() {
		return d[0];
	}
	public String getName() {
		return super.getName()
	}
	public boolean connect() {
		return super.connect()
	}
	
	public void disconnect() {
		 super.disconnect()
	}

	@Override 
	public String toString(){
		return ""+d+" "+dataTmp 
	}
}

def dev = DeviceManager.getSpecificDevice( "hidDevice",{
	//If the device does not exist, prompt for the connection
	def simp = null;
	
	HashSet<InetAddress> addresses = UDPSimplePacketComs.getAllAddresses("hidDevice");
	if (addresses.size() < 1) {
			simp = new SimpleServoHID(0x16C0 ,0x0486) 
	}else{
		simp = new SimpleServoUDP(addresses.toArray()[0])
				simp.setReadTimeout(100);
	}
	HIDSimpleComsDevice d = new HIDSimpleComsDevice(simp)
	d.connect(); // Connect to it.
	if(simp.isVirtual()){
		println "\n\n\nDevice is in virtual mode!\n\n\n"
	}

	LinkFactory.addLinkProvider("hidfast",{LinkConfiguration conf->
				println "Loading link "
				return new HIDRotoryLink(d,conf)
		}
	)
	println "Connecting new device: "+d
	return d
})

def gc= DeviceManager.getSpecificDevice(args[2],{
		def controllers =GameControllerLocal.getc(args[2])
		def control = controllers[0]
		if(control==null)
			return null
		control.connect()

		while(control.getName().getBytes().size()==0){
			println "Waiting for device name..."
			Thread.sleep(500)// wait for the name packet to be sent
			//String n = control.getName();
		}
		String n = control.getName();
		println "Device named ="+n.getBytes()+" " + n
		for (def method : control.getClass().getMethods()) {
			if(method.getName().contains("connect"))
				println method.getName()
		}
		return control;
	})
def cat =DeviceManager.getSpecificDevice( "MediumKat",{
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
		 			imuDataValues[0],//Double xAcceleration,
		 			imuDataValues[1],//Double yAcceleration
			 		imuDataValues[2],//,Double zAcceleration
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
