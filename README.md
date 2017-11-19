# mBot simulation in V-REP

#### Note: you need three programs for this to work: V-REP PRO (EDU) from [Coppelia Robotics GmbH](http://www.coppeliarobotics.com/index.html), mBlock from [Makeblock Co., Ltd.](http://www.makeblock.com/) and virtual serial port emulator, I use the one from [ETERLOGIC.COM](http://www.eterlogic.com/Products.VSPE.html)

---

This is mBot model for simulation in V-REP. It works by receiving commands from mBlock through a virtual serial port and executing them in V-REP on a virtual robot. The model contains the lua script which takes care of serial communication and executes commands. It is easy to install and use, all programs that you need are free (for educational purpose).

## Getting started
First you have to download and install programs that you will need to simulate your virtual mBot robot.

| Program name | Description | Download link |
| :-------------: | :-------------: | :----: |
| V-REP | Robot simulation platform | [coppeliarobotics.com](http://www.coppeliarobotics.com/downloads.html) |
| mBlock | Graphical programming environment | [makeblock.com](http://learn.makeblock.com/en/software/)|
| VSPE | Virtual serial port emulator | [eterlogic.com](http://www.eterlogic.com/Downloads.html)

## Virtual serial port emulator
Before starting the simulation you have to create a virtual serial port that V-REP and mBlock can connect to. Open VSPE and click on ``Create new device`` button in the toolbar or create a new device through the menu ``Device->Create``. Leave the device type on ``Connector``, click ``Next``, select a port that does not exist in your system (e.g. if you have a hardware serial port COM1 than select COM2 or some other port) and click ``Finish``. New virtual serial port should be created and initialized.

## V-REP
This is the program in which you will run your simulation. You can use .ttt scene files provided or just the model file ``mBot.ttm`` in your own scenes. After you load a scene or the mBot model, you have to open the ``Script Parameters`` dialog by clicking on the icon located on the right of the child script icon attached on a dummy object called ``mBotScript`` and select ``Com port name`` parameter. You can access those by expanding the treeview node called ``mBot`` in the scene hierarchy. Now, for the value, enter the name of the port that you created using virtual serial port emulator (e.g.``COM2``). Close the dialog and run the simulation.

## mBlock
mBlock is a graphical programming environment based on Scratch. Here you can make a program for your virtual robot as you would do for the real mBot except you can't upload programs, you have to use the command mode instead (Scratch mode). Before you can run your program, you have to connect mBlock to the virtual serial port that you created using VSPE (the same one that V-REP is connected to). Go to the menu ``Connect->Serial Port`` and choose that port. Now you can start your program.

## Simulated devices
Supported devices are motors, ultrasonic sensor, line follower sensors, LEDs on board and seven segment display. I'm still working on LED matrix and servo so in the next update those will be available too. This mBot model does not support Makeblock devices that can not be simulated in V-REP. For example, V-REP doesn't support sound so you can't use the ``Play tone`` block in mBlock. Maybe in the future versions I will make the code in C/C++ as a V-REP plugin to be able to support devices that are not supported in V-REP by default.

## Using mBot script in other V-REP models
With this script you can also simulate other (or your own) V-REP models. For that, you have to load your model in V-REP (or select one from the ``Model browser`` on the left), move the dummy object called ``mBotScript`` located inside of mBot model to the hierarchy of your loaded model using drag and drop. Then open the ``Script parameters`` dialog (descirbed above), select the joint name parameters one by one and set their values to the names of your model's joint objects. Joint name parameters are ``Left motor joint name`` and ``Right motor joint name``. Do the same for all name parameters like ``Ultrasonic sensor name``, ``Left line follower sensor name``, ``Right line follower sensor name`` etc. For the sensors or joints that you will not use you have to delete their value (set to blank) or the script will throw an error and stop. If you choose the model which already contains the script that controls your model's joints, you have to remove or disable that script. You can disable a script in V-REP in the ``Scripts`` dialog. Go to the menu ``Tools->Scripts`` and in the dialog select the script you want to disable than click on ``Disabled``.
