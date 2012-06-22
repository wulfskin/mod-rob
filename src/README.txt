/** \mainpage
	This is the documentation of the software from the team _Kick-Ass_ of the special course _31384 Modular Robotics 2012_
	at the Technical University of Denmark. The team consists of the following four team members:
	<ul>
		<li><a href="mailto:s052332@student.dtu.dk">Dennis Hellner (s052332)</a></li>
		<li><a href="mailto:s080147@student.dtu.dk">Hans-Peter Wolf (s111032)</a></li>
		<li><a href="mailto:s080147@student.dtu.dk">Rune Klarskov Pagter (s080147)</a></li>
		<li><a href="mailto:s104169@student.dtu.dk">Walter Gambelunghe (s104169)</a></li>
	</ul>
	The documentation covers three different software projects reflecting four different robots. Each robots was built
	using the <a href="http://www.robotis.com/xe/bioloid_en">Bioloid Comprehensive Robot Kit</a> feauturing an 
	<a href="http://www.atmel.com/devices/atmega2561.aspx">Atmel ATmega2561</a> microcontroller. More information on each
	project is found in the respective source code. Details about coding-style conventions can be found in src/styleguide.txt.
 	
	The four different projects (applications) are:
	<ol>
		<li>
			<b><i>Wally</i>: A wheeled autonomous robot (1-wheeled-rob/main.c):</b><br>
			The goal of this project is to create a robot that can navigate _autonomously_ through a very easy course
			using wheels.<br>
			<i>Responsibility: Walter Gambelunghe.</i>
		</li>
		<li>
			<b><i>Squid I/II</i>: A non-wheeled remote-controlled and autonomous robot (2-nonwheeled-rob/main.c):</b><br>
			The goal of this project is to create a robot that can navigate either _remote-controlled_ or _autonomously_ through
			a very easy course using only non-rotating movements. Both projects share the same source code whit an option to
			select the desired operation mode.<br>
			<i>Responsibility: Hans-Peter Wolf (remote-controlled) and Dennis Hellner (autonomous).</i>
		</li>
		<li>
			<b><i>Shark</i>: A toy robot (4-toy-robot/main.c):</b><br>
			The goal of this project is to create a toy robot. That is a robot that can be used as a toy.<br>
			<i>Responsibility: Rune Klarskov Pagter.</i>
		</li>
	</ol>
	Besides the software of each project there is a firmware layer that provides common functions that are used by all four
	projects. The firmware layer provides the following functionalities:
	<ul>
		<li>Common helper functions (macro.h)</li>
		<li>Common error reporting functions (error.h)</li>
		<li>In- and output functions including LEDs, buttons, microphone and buzzer (io.h)</li>
		<li>Dynamixel motor control functions (motor.h)</li>
		<li>Sensor usage functions (sensor.h)</li>
		<li>Serial communication helper functions (serial.h and serialzigbee.h)</li>
		<li>Timer interface functions (timer.h)</li>
	</ul>
	The software is developed using <a href="http://www.atmel.com/microsite/atmel_studio6/">Atmel Studio 6</a> and 
	<a href="http://winavr.sourceforge.net/">WinAVR</a> and its only dependency is the Robotis Dynamixel library which is also
	provided with the source code. As there are no makefiles included it is recommend to use the provided Atmel Studio 6 projects
	to compile. The source code is also found on <a href="https://github.com/wulfskin/mod-rob">GitHub/mod-rob</a>. The following
	image provides an overview about the different software and hardware layers:
	\image html layer_overview.jpg
 */