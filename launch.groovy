@Grab(group='org.hid4java', module='hid4java', version='0.5.0')


import org.hid4java.*
import org.hid4java.event.*;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import Jama.Matrix;
class PacketProcessor{
	ByteOrder be =ByteOrder.LITTLE_ENDIAN; 
	int packetSize = 64
	int numFloats =packetSize-4

	int getId(byte [] bytes){
		return ByteBuffer.wrap(message).order(be).getInt(0);
	}
	int[] parse(byte [] bytes){
		int[] returnValues = new int[ numFloats];
		
		//println "Parsing packet"
		for(int i=0;i<numFloats;i++){
			int baseIndex = i+4;
			//returnValues[i] = ByteBuffer.wrap(bytes).order(be).getFloat(baseIndex);
			returnValues[i] = bytes[baseIndex]
			if(returnValues[i] <0){
				returnValues[i]+=256
			}
		}
			
		return returnValues
	}
	byte[] command(int idOfCommand, int []values){
		byte[] message = new byte[packetSize];
		ByteBuffer.wrap(message).order(be).putInt(0,idOfCommand).array();
		for(int i=0;i<numFloats && i< values.length;i++){
			int baseIndex = i+4;
			//ByteBuffer.wrap(message).order(be).putFloat(baseIndex,values[i]).array();
			message[baseIndex]=(byte)values[i]
		}
		return message
	}
	
}

public class PacketType{
	private final Integer idOfCommand;
	int [] downstream = new int[60]
	int [] upstream = new int[60]
	boolean done=false;
	boolean started = false;
	public PacketType(Integer id){
		idOfCommand=id;
	}
	
}

public class HIDSimpleComsDevice extends NonBowlerDevice{
	HashMap<Integer,ArrayList<Runnable>> events = new HashMap<>()
	HidServices hidServices = null;
	int vid =0 ;
	int pid =0;
	HidDevice hidDevice=null;
	public PacketProcessor processor= new PacketProcessor();
	boolean HIDconnected = false;
	PacketType pollingPacket = new PacketType(37);
	PacketType pidPacket = new PacketType(65);
	PacketType PDVelPacket = new PacketType(48);
	PacketType SetVelocity = new PacketType(42);
	
	ArrayList<PacketType> processQueue = [] as ArrayList<PacketType>
	
	public HIDSimpleComsDevice(int vidIn, int pidIn){
		// constructor
		vid=vidIn
		pid=pidIn
		setScriptingName("hidbowler")
	}

	public void pushPacket(def packet){
		packet.done=false;
		packet.started = false;
		processQueue.add(packet)
		while(packet.done==false){
			Thread.sleep(1)
		}
	}
	void removeEvent(Integer id, Runnable event){
		if(events.get(id)==null){
			events.put(id,new ArrayList<Runnable>())
		}
		events.get(id).remove(event)
	}
	void addEvent(Integer id, Runnable event){
		if(events.get(id)==null){
			println "Creating "+id+" list"
			events.put(id,new ArrayList<Runnable>())
		}
		
		events.get(id).add(event)
		println "Adding "+id+" event "+event+" size "+events.get(id).size()
	}
	@Override
	public  void disconnectDeviceImp(){		
		HIDconnected=false;
		println "HID device Termination signel shutdown"
	}
	private void process(def packet){
		packet.started=true
		try{
			if(hidDevice!=null){
				//println "Writing packet"
				try{
					byte[] message = processor.command(packet.idOfCommand,packet.downstream)
					//println "Writing: "+ packet.downstream
					int val = hidDevice.write(message, message.length, (byte) 0);
					if(val>0){
						int read = hidDevice.read(message, 1000);
						if(read>0){
							//println "Parsing packet"
							
							def up=processor.parse(message)
							for(int i=0;i<packet.upstream.length;i++){
								packet.upstream[i]=up[i];
							}
							//println "read: "+ packet.upstream
						}else{
							println "Read failed"	
						}
						
					}
				}catch (Throwable t){
					t.printStackTrace(System.out)
					disconnect()
				}
			}else{
				//println "Simulation"
				for(int j=0;j<packet.downstream.length&&j<packet.upstream.length;j++){
					packet.upstream[j]=packet.downstream[j];
			}
				
			}
			//println "updaing "+upstream+" downstream "+downstream
			ArrayList<Runnable> eventRunnables = events.get(packet.idOfCommand)
			if(eventRunnables!=null){
				//println "Command "+packet.idOfCommand+" has "+eventRunnables.size()
				for(int i=0;i<eventRunnables.size();i++){
					Runnable e=eventRunnables.get(i)
					if(e!=null){
						try{
							e.run()
						}catch (Exception |Error t){
							t.printStackTrace(System.out)							
						}
					}
				}
			}
		}catch (Throwable t){
					t.printStackTrace(System.out)
		}
		packet.done=true
	}
	
	@Override
	public  boolean connectDeviceImp(){
		if(hidServices==null)
			hidServices = HidManager.getHidServices();
		// Provide a list of attached devices
		hidDevice=null
		for (HidDevice h : hidServices.getAttachedHidDevices()) {
		  if(h.isVidPidSerial(vid, pid, null)){
		  	if(hidDevice== null){
		  	  hidDevice=h
			 
			  hidDevice.open();
			  System.out.println("Found! "+hidDevice);
		  	}else{
		  		System.out.println("Already opened! this matches too.. "+h);
		  	}
			 
		  }
		}
		HIDconnected=true;
		Thread.start{
			println "Starting HID Thread"
			while(HIDconnected){
				//println "loop"
				try{
					Thread.sleep(1)
					if(pollingPacket!=null){
						pollingPacket.done=false;
						pollingPacket.started = false;
						process(pollingPacket)
					}
					while(processQueue.size()>0){
						try{
							def temPack =processQueue.remove(0)
							if(temPack!=null){
								println "Processing "+temPack
								process(temPack)
							}
						}catch(Exception e){
							e.printStackTrace()
						}
						
					}
				}catch(Exception e){
					e.printStackTrace()
				}

				
			}
			if(hidDevice !=null){
				hidDevice.close();
			}
			if(hidServices!=null){
				// Clean shutdown
				hidServices.shutdown();
			}
			println "HID device clean shutdown"
		 }
		//throw new RuntimeException("No HID device found")
	}
	void setValue(int index,int position){
		pollingPacket.downstream[index] = position
	}
	int getValue(int index){
		return pollingPacket.upstream[index]
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
	private static final Integer command =37
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
		c.addEvent(command,{
			int val= getCurrentPosition();
			if(lastPushedVal!=val){
				//println "Fire Link Listner "+index+" value "+getCurrentPosition()
				fireLinkListener(val);
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


def dev = DeviceManager.getSpecificDevice( "hidDevice",{
	//If the device does not exist, prompt for the connection
	
	HIDSimpleComsDevice d = new HIDSimpleComsDevice( 0x16c0 ,0x0486 )
	d.connect(); // Connect to it.
	LinkFactory.addLinkProvider("hidfast",{LinkConfiguration conf->
				println "Loading link "
				return new HIDRotoryLink(d,conf)
		}
	)
	println "Connecting new device: "+d
	return d
})

def cat =DeviceManager.getSpecificDevice( "jaguar",{
	//If the device does not exist, prompt for the connection
	
	MobileBase m = MobileBaseLoader.fromGit(
		"https://github.com/keionbis/SmallKat.git",
		"Bowler/cat.xml"
		)
	if(m==null)
		throw new RuntimeException("Arm failed to assemble itself")
	println "Connecting new device robot arm "+m
	return m
})

def gameController = ScriptingEngine.gitScriptRun(
            "https://gist.github.com/e26c0d8ef7d5283ef44fb22441a603b8.git", // git location of the library
            "LoadGameController.groovy" , // file to load
            // Parameters passed to the funcetion
            ["Game*"]
            )
if(gameController==null){
	return 
}

byte [] data = gameController.getData() 
double toSeconds=0.03//100 ms for each increment
          
while (!Thread.interrupted()){
	Thread.sleep((toSeconds*1000))
	double xdata = data[0]
	double rzdata = data[1]
	if(xdata<0)
		xdata+=256
	if(rzdata<0)
		rzdata+=256
		
	double displacement = 5.0*xdata/255.0
	double rot = 5.0*rzdata/255.0
	TransformNR move = new TransformNR(displacement,0,0,new RotationNR(0,rot,0))
	cat.DriveArc(move, toSeconds);
}
