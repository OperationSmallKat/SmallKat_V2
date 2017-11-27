//If your DyIO is using a lower voltage power source, you need to disable the brownout detect
def myDyIO =DeviceManager.getSpecificDevice( "dyio",{
	//If the device does not exist, prompt for the connection
	SerialConnection s =new SerialConnection("/dev/ttyACM0");
	DyIO m = new DyIO(s)
	return m
})


def cat =DeviceManager.getSpecificDevice( "jaguar",{
	//If the device does not exist, prompt for the connection
	
	MobileBase m = BowlerStudio.loadMobileBaseFromGit(
		"https://github.com/keionbis/SmallKat.git",
		"bowler/cat.xml"
		)
	if(m==null)
		throw new RuntimeException("Arm failed to assemble itself")
	println "Connecting new device robot arm "+m
	return m
})

return null