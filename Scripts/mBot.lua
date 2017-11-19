-- Implementation of Orion protocol and part of mBot's firmware. This script executes commands sent from mBlock in Scratch mode.
-- Implementacija Orion protokola i dijela mBot firmware-a, izvršavanje blokova iz mBlock-a za koje simulacija ima smisla.
-- by Nenad Stajić 2017.

	-- Parameters
	robotIsOn = simGetScriptSimulationParameter(sim_handle_self,"Robot power",false)
	comPortName = simGetScriptSimulationParameter(sim_handle_self,"Com port name",false)
	motorSpeedCoefficient = simGetScriptSimulationParameter(sim_handle_self,"Motor speed coefficient",false)
	motorLeftJointName = simGetScriptSimulationParameter(sim_handle_self,"Left motor joint name",false)
	motorRightJointName = simGetScriptSimulationParameter(sim_handle_self,"Right motor joint name",false)
	ultrasonicSensorName = simGetScriptSimulationParameter(sim_handle_self,"Ultrasonic sensor name",false)
	leftLineFollowerName = simGetScriptSimulationParameter(sim_handle_self,"Left line follower sensor name",false)
	rightLineFollowerName = simGetScriptSimulationParameter(sim_handle_self,"Right line follower sensor name",false)
	leftLineFollowerLEDName = simGetScriptSimulationParameter(sim_handle_self,"Left line follower LED name",false)
	rightLineFollowerLEDName = simGetScriptSimulationParameter(sim_handle_self,"Right line follower LED name",false)
	sevSegDisplayPrecision = simGetScriptSimulationParameter(sim_handle_self,"Seven segment display precision",false)
	LED_power_name = simGetScriptSimulationParameter(sim_handle_self,"LED power name", false)
	LED_battery_name = simGetScriptSimulationParameter(sim_handle_self,"LED battery name", false)
	LED_battery_power_name = simGetScriptSimulationParameter(sim_handle_self,"LED battery power name", false)
	LED_RGB_left_name = simGetScriptSimulationParameter(sim_handle_self,"LED RGB left name", false)
	LED_RGB_right_name = simGetScriptSimulationParameter(sim_handle_self,"LED RGB right name", false)
	debugConsoleOn = simGetScriptSimulationParameter(sim_handle_self,"Debug console output",false)
	
	-- Robot parts
	if motorLeftJointName == "" then
		leftWheelJoint = nil
	else
		leftWheelJoint = simGetObjectHandle(motorLeftJointName)
	end
	if motorRightJointName == "" then
		rightWheelJoint = nil
	else
		rightWheelJoint = simGetObjectHandle(motorRightJointName)
	end
	if ultrasonicSensorName == "" then
		ultrasonicSensor = nil
	else
		ultrasonicSensor = simGetObjectHandle(ultrasonicSensorName)
	end
	if leftLineFollowerName == "" then
		leftLineFollower = nil
	else
		leftLineFollower = simGetObjectHandle(leftLineFollowerName)
	end
	if rightLineFollowerName == "" then
		rightLineFollower = nil
	else
		rightLineFollower = simGetObjectHandle(rightLineFollowerName)
	end
	if leftLineFollowerLEDName == "" then
		LED_LineFollower_left = nil
	else
		LED_LineFollower_left = simGetObjectHandle(leftLineFollowerLEDName)
	end
	if rightLineFollowerLEDName == "" then
		LED_LineFollower_right = nil
	else
		LED_LineFollower_right = simGetObjectHandle(rightLineFollowerLEDName)
	end
	if LED_power_name == "" then
		LED_Power = nil
	else
		LED_Power = simGetObjectHandle(LED_power_name)
	end
	if LED_battery_name == "" then
		LED_Battery = nil
	else
		LED_Battery = simGetObjectHandle(LED_battery_name)
	end
	if LED_battery_power_name == "" then
		LED_BatteryPower = nil
	else
		LED_BatteryPower = simGetObjectHandle(LED_battery_power_name)
	end
	if LED_RGB_left_name == "" then
		LED_RGB_left = nil
	else
		LED_RGB_left = simGetObjectHandle(LED_RGB_left_name)
	end
	if LED_RGB_right_name == "" then
		LED_RGB_right = nil
	else
		LED_RGB_right = simGetObjectHandle(LED_RGB_right_name)
	end	
	
	-- Init
    comPort = "\\\\.\\"..comPortName  --"\\\\.\\COM2" za Windows
    baudrate = 115200
    if robotIsOn then
		serial = simSerialOpen(comPort, baudrate)
		if LED_Power ~= nil then simSetLightParameters(LED_Power, 1, nil, nil, nil) end
		if LED_Battery ~= nil then simSetLightParameters(LED_Battery, 1, nil, nil, nil) end
		if LED_BatteryPower ~= nil then simSetLightParameters(LED_BatteryPower, 1, nil, nil, nil) end
	end
    if debugConsoleOn then 
		console = simAuxiliaryConsoleOpen("Aux Console", 500, 0x10, {80, 150}, {400, 800}) 
	end
	simSetThreadAutomaticSwitch(false)

	-- Action
	GET = 1
	RUN = 2
	RESET = 4
	START = 5
	
	-- Devices
	ULTRASONIC_SENSOR = 1
	TEMPERATURE_SENSOR = 2
	LIGHT_SENSOR = 3
	POTENTIONMETER = 4
	JOYSTICK = 5
	GYRO = 6
	SOUND_SENSOR = 7
	RGBLED = 8
	SEVSEG = 9
	MOTOR = 10
	SERVO = 11
	ENCODER = 12
	IR = 13
	IRREMOTE = 14
	PIRMOTION = 15
	INFRARED = 16
	LINEFOLLOWER = 17
	IRREMOTECODE = 18
	SHUTTER = 20
	LIMITSWITCH = 21
	BUTTON = 22
	HUMITURE_SENSOR = 23
	FLAME_SENSOR = 24
	GAS_SENSOR = 25
	COMPASS_SENSOR = 26
	DIGITAL = 30
	ANALOG = 31
	PWM = 32
	SERVO_PIN = 33
	TONE = 34
	BUTTON_INNER = 35
	LEDMATRIX = 41
	TIMER = 50
	TOUCH_SENSOR = 51
	
	-- Ports
	M1 = 0x09
	M2 = 0x0a
	
	-- Misc
	isStartReading = false 
	commandBlock = {}
	
	serialBuffer = {}
	cByte = 0
	cPrev = 0
	index = 0
	dataLen = 0
	commandIndex = 0
	
	-- Debug
	function DebugLog(text)
		if debugConsoleOn then simAuxiliaryConsolePrint(console, text.."\n") end
	end
	testNo = 0
	DebugLog("Serial port handle: "..serial)
	DeviceDescription = {}
	DeviceDescription[1] = "Ultrasonic sensor"
	DeviceDescription[2] = "Temperature sensor"
	DeviceDescription[3] = "Light sensor"
	DeviceDescription[4] = "Potentiometer"
	DeviceDescription[5] = "Joystick"
	DeviceDescription[6] = "Gyro"
	DeviceDescription[7] = "Sound sensor"
	DeviceDescription[8] = "RGB LED"
	DeviceDescription[9] = "Seven segment display"
	DeviceDescription[10] = "Motor"
	DeviceDescription[11] = "Servo"
	DeviceDescription[12] = "Encoder"
	DeviceDescription[13] = "IR"
	DeviceDescription[14] = "IR remote"
	DeviceDescription[15] = "PIR motion"
	DeviceDescription[16] = "Infrared"
	DeviceDescription[17] = "Line follower"
	DeviceDescription[18] = "IR remote code"
	DeviceDescription[19] = "Unknown device (19)"
	DeviceDescription[20] = "Shutter"
	DeviceDescription[21] = "Limit switch"
	DeviceDescription[22] = "Button"
	DeviceDescription[23] = "Humiture sensor"
	DeviceDescription[24] = "Flame sensor"
	DeviceDescription[25] = "Gas sensor"
	DeviceDescription[26] = "Compass sensor"
	DeviceDescription[27] = "Unknown device (27)"
	DeviceDescription[28] = "Unknown device (28)"
	DeviceDescription[29] = "Unknown device (29)"
	DeviceDescription[30] = "Digital"
	DeviceDescription[31] = "Analog"
	DeviceDescription[32] = "PWM"
	DeviceDescription[33] = "Servo pin"
	DeviceDescription[34] = "Tone"
	DeviceDescription[35] = "Button inner"
	DeviceDescription[36] = "Unknown device (36)"
	DeviceDescription[37] = "Unknown device (37)"
	DeviceDescription[38] = "Unknown device (38)"
	DeviceDescription[39] = "Unknown device (39)"
	DeviceDescription[40] = "Unknown device (40)"
	DeviceDescription[41] = "LED matrix"
	DeviceDescription[50] = "Timer"
	DeviceDescription[51] = "Touch sensor"
	
	-- Conversions
	function hex2float (b4, b3, b2, b1) -- Little Endian (DCBA)
		local sign = b1 > 0x7F
		local expo = (b1 % 0x80) * 0x2 + math.floor(b2 / 0x80)
		local mant = ((b2 % 0x80) * 0x100 + b3) * 0x100 + b4

		if sign then
			sign = -1
		else
			sign = 1
		end

		local n

		if mant == 0 and expo == 0 then
			n = sign * 0.0
		elseif expo == 0xFF then
			if mant == 0 then
				n = sign * math.huge
			else
				n = 0.0/0.0
			end
		else
			n = sign * math.ldexp(1.0 + mant / 0x800000, expo - 0x7F)
		end
		return n
	end
	
	function float2hex (n)
		if n == 0.0 then return 0.0 end

		local sign = 0
		if n < 0.0 then
			sign = 0x80
			n = -n
		end

		local mant, expo = math.frexp(n)
		local hext = {}

		if mant ~= mant then
			hext[#hext+1] = string.char(0xFF, 0x88, 0x00, 0x00)

		elseif mant == math.huge or expo > 0x80 then
			if sign == 0 then
				hext[#hext+1] = string.char(0x7F, 0x80, 0x00, 0x00)
			else
				hext[#hext+1] = string.char(0xFF, 0x80, 0x00, 0x00)
			end

		elseif (mant == 0.0 and expo == 0) or expo < -0x7E then
			hext[#hext+1] = string.char(sign, 0x00, 0x00, 0x00)

		else
			expo = expo + 0x7E
			mant = (mant * 2.0 - 1.0) * math.ldexp(0.5, 24)
			hext[#hext+1] = string.char(sign + math.floor(expo / 0x2), (expo % 0x2) * 0x80 + math.floor(mant / 0x10000),
										math.floor(mant / 0x100) % 0x100, mant % 0x100)
		end
		return hext[1]
	end
	
	function BytesToShort(msb, lsb)
		strMsb = string.format("%x",msb)
		strLsb = string.format("%x",lsb)
		if #strMsb < 2 then strMsb = "0"..strMsb end
		if #strLsb < 2 then strLsb = "0"..strLsb end
		unsigned = tonumber(strMsb..strLsb,16)
		if unsigned > 32767 then return (unsigned-2^15)-2^15 end
		return unsigned
	end
	
	function GetByte()
		return table.remove(serialBuffer, 1)
	end
	
	function ReadFloat(cblockIndex)
		local b1,b2,b3,b4 = commandBlock[cblockIndex], commandBlock[cblockIndex + 1], commandBlock[cblockIndex + 2], commandBlock[cblockIndex + 3]
		return hex2float(b1, b2, b3, b4)
	end
	
	function ReadShort(cblockIndex)
		return BytesToShort(commandBlock[cblockIndex+1], commandBlock[cblockIndex]) -- Little endian
	end
	
	-- Pošalji bajt na serijski port
	function WriteByte(data)
		simSerialSend(serial, string.char(data))
	end
	
	function Write16bit(b1, b2)
		simSerialSend(serial, string.char(b1, b2))
	end
	
	function Write32bit(b1, b2, b3, b4)
		simSerialSend(serial, string.char(b1, b2, b3, b4))
	end
	
	-- Tip podatka u odgovoru se šalje prije samog podatka: 1 byte 2 float 3 short 4 len+string 5 double
	function SendByte(data)
		WriteByte(1)
		WriteByte(data)
	end
	
	function SendFloat(data)
		if data == nil then return end
		WriteByte(2)
		local b4, b3, b2, b1
		if data > 0 then
			 b4, b3, b2, b1 = string.byte(float2hex(data), 1, 4)
			else
				b1 = 0
				b2 = 0
				b3 = 0
				b4 = 0
		end
		Write32bit(b1, b2, b3, b4)
	end
	
	function WriteHead()
		Write16bit(0xff, 0x55)
	end

	function WriteEnd()
		Write16bit(0x0d, 0x0a)
	end
	
	
	function HandleSerialData()
		-- 0xff 0x55 length index action device payload
		
		while table.getn(serialBuffer) > 0 do
			
			cByte = GetByte()
			if cByte == nil then break end
			
			if cByte == 0x55 and not isStartReading then
				if cPrev == 0xff then 
					isStartReading = true
					index = 1
				end				
			else
				cPrev = cByte
				if isStartReading then 
					if index == 2 then
						dataLen = cByte
					elseif index > 2 then
						dataLen = dataLen - 1
					end
					commandBlock[index] = cByte
				end
			end
			
			index = index + 1
			
			if index > 51 then
				index = 0
				isStartReading = false
			end
			
			if isStartReading and dataLen == 0 and index > 3 then
				isStartReading = false
				ParseData()
				index = 0
			end
			
		end
		
	end

	function ParseData()
		isStartReading = false
		commandIndex = commandBlock[3]
		action = commandBlock[4]
		device = commandBlock[5]
		
		if action == GET then
			--if device ~= ULTRASONIC_SENSOR then
				WriteHead()
				WriteByte(commandIndex)
			--end
			ReadSensor(device)
			WriteEnd()
		
		elseif action == RUN then
			RunModule()
			WriteHead()
			WriteEnd()
		
		elseif action == RESET then
			WriteHead()
			WriteEnd()
		
		elseif action == START then
			WriteHead()
			WriteEnd()
		
		end
	end
	
	function RunModule()
		local port = commandBlock[6]
		DebugLog("\nRun module, device: "..DeviceDescription[device]..", port: "..port)
		if device == MOTOR then
			local speed = ReadShort(7)
			if port == M1 then RunMotorL(speed)
			elseif port == M2 then RunMotorR(speed) end
		
		elseif device == JOYSTICK then
			local leftSpeed = ReadShort(6)
			local rightSpeed = ReadShort(8)
			RunMotorL(leftSpeed)
			RunMotorR(rightSpeed)
		
		elseif device == RGBLED then
			local slot = commandBlock[7]
			local idx = commandBlock[8]
			local r = commandBlock[9]
			local g = commandBlock[10]
			local b = commandBlock[11]
			LedSetColor(port,slot,idx, r, g, b)
		
		elseif device == SERVO then
			slot = commandBlock[7]
			Servo(slot, commandBlock[8])
			
		elseif device == SEVSEG then
			v = ReadFloat(7)
			SevSegDisplay(v)

		else DebugLog("This device is not simulated in RUN mode.")
		end
	end
	
	function ReadSensor()
		local port = commandBlock[6]
		DebugLog("\nRead sensor, device: "..DeviceDescription[device]..", port: "..port)
		if device == ULTRASONIC_SENSOR then
			SendFloat(ReadUltrasonicSensor(port))
			
			elseif device == LINEFOLLOWER then
				SendFloat(ReadLineFollower(port))
			
			-- Add more sensor types here
		
			else DebugLog("This device is not simulated in GET mode.")
		
		end		
	end
	
	-- Simulation
	function RunMotorL(speed)
		-- Mblock u komandi za lijevi motor šalje invertiranu vrijednost tj. -speed (zbog orijentacije motora u mBot-u)
		DebugLog("Run Motor M1 (left), speed: "..speed)
		if leftWheelJoint == nil then
			DebugLog("Left wheel joint is not defined.")
			return
		end
		simSetJointTargetVelocity(leftWheelJoint, -speed * motorSpeedCoefficient)
	end
	
	function RunMotorR(speed)
		DebugLog("Run Motor M2 (right), speed: "..speed)
		if rightWheelJoint == nil then
			DebugLog("Right wheel joint is not defined.")
			return
		end
		simSetJointTargetVelocity(rightWheelJoint, speed * motorSpeedCoefficient)
	end
	
	function LedSetColor(port, slot, ledIdx, red, green, blue)
		DebugLog("Set LED color at port "..port..", slot "..slot..", index "..ledIdx..", color: r"..red.." g"..green.." b"..blue)
		if LED_RGB_right == nil then
			DebugLog("Right RGB LED is not defined.")
			return
		end
		if LED_RGB_left == nil then
			DebugLog("Left RGB LED is not defined.")
			return
		end
		local state = 1
		if red == 0 and green == 0 and blue == 0 then state = 0 end
		if ledIdx == 0 or ledIdx == 1 then
			simSetLightParameters(LED_RGB_right, state, nil, {red, green, blue}, {red, green, blue})
		end
		if ledIdx == 0 or ledIdx == 2 then
			simSetLightParameters(LED_RGB_left, state, nil, {red, green, blue}, {red, green, blue})
		end
	end
	
	function Servo(slot, angle)
		DebugLog("Set servo at slot "..slot.." to angle: "..angle)
		-- Not implemented
	end
	
	function SevSegDisplay(value)
		DebugLog("Seven segment display, number: "..value)
		simSetFloatSignal("SevSegPrecision"..sevSegDisplayPrecision, value)
	end
	
	function ReadLineFollower(port)
		-- Response value is 0~3, in binary 00, 10, 01, 11, the higher digit is for left sensor
		-- 0 - Both Sensors are over the line
		-- 1 - Left Sensor is over the line / Right Sensor is off of the line
		-- 2 - Left Sensor is off the line / Right Sensor is over of the line
		-- 3 - Both Sensors are off of the line
		if leftLineFollower == nil or rightLineFollower == nil then
			DebugLog("Line follower sensor is not defined.")
			return
		end
		if LED_LineFollower_left == nil or LED_LineFollower_right == nil then
			DebugLog("Left and right line follower LEDs are not defined.")
			return
		end
		leftSensor = simReadVisionSensor(leftLineFollower)
		rightSensor = simReadVisionSensor(rightLineFollower)
		simSetLightParameters(LED_LineFollower_left, leftSensor, nil, nil, nil)
		simSetLightParameters(LED_LineFollower_right, rightSensor, nil, nil, nil)
		value = leftSensor * 2 + rightSensor
		DebugLog("Read linefollower sensor at port "..port..", value: "..value.."\nLeft sensor: "..leftSensor.."\nRight sensor: "..rightSensor)
		return value
	end
	
	function ReadUltrasonicSensor(port)
		if ultrasonicSensor == nil then
			DebugLog("Ultrasonic sensor is not defined.")
			return
		end
		local result, distance = simReadProximitySensor(ultrasonicSensor)
		if result > 0 then
				distance = distance * 100	-- Convert distance from m to cm
				DebugLog("Read ultrasonic sensor at port "..port..", value: "..distance.."cm")
				return distance
			
			elseif result == 0 then
				DebugLog("Read ultrasonic sensor at port "..port..", no detection.")
				return 0
				
			else
				DebugLog("Reading ultrasonic sensor at port "..port.." failed.")
				return 0
		end
	end
	
	-- Main loop
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop and robotIsOn do       
		buffer = simSerialRead(serial, 255, false, 0, 100)
		if buffer ~= nil then
			length = #buffer
			local n = 1
			while n <= length do
				table.insert(serialBuffer, string.byte(buffer,n))
				n = n + 1
			end
		end
		if table.getn(serialBuffer) > 0 then
			HandleSerialData()
		end
		simSwitchThread()
    end

-- Clean up
if serial > -1 then
	simSerialClose(serial)
end

if LED_Power ~= nil then simSetLightParameters(LED_Power, 0, nil, nil, nil) end
if LED_Battery ~= nil then simSetLightParameters(LED_Battery, 0, nil, nil, nil) end
if LED_BatteryPower ~= nil then simSetLightParameters(LED_BatteryPower, 0, nil, nil, nil) end
if LED_RGB_left ~= nil then simSetLightParameters(LED_RGB_left, 0, nil, nil, nil) end 
if LED_RGB_right ~= nil then simSetLightParameters(LED_RGB_right, 0, nil, nil, nil)end
