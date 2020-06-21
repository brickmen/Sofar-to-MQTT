---
title: Sofar HYD6000-ES ModbusRead
author: Andy Whittaker
tags: 'Modbus, Node-RED, Emon, RS485, Sofar Inverter, Hybrid, HYD6000-ES'
categories: Solar PV
date: '2020-05-28'

---

<h1 id="sofar-hyd6000-es-modbusread">Sofar HYD6000-ES ModbusRead</h1>
<p>This is a very badly written Python Script to Request and Read Data from the HYD6000 Inverter RS485 Port and Write To EmonCMS (or EmonPi).</p>
<p>Since we require the script to be called at regular intervals, I have also included a very simple Node-RED script to call the Python code (currently) at 30 second intervals.</p>
<h2 id="interfacing-with-the-hyd6000">Interfacing with the HYD6000</h2>
<p>We require to interface between the Raspberry Pi and the Sofar HYD6000 inverter via RS485. On the bottom of the HYD6000 you can find the interface connections.</p>
<p><img src="https://www.andywhittaker.com/img/HYD6000-RS485-01.jpg" alt="Sofar HYD6000 Interfaces"></p>
<p>The CAN/485m connection goes off to the batteries. However, the next interface available is the RS485s connection, Sofar even gives you the correct connector to plug-in (it looks to me like a Phoenix/ Weidmüller Type).</p>
<p>We now need a cheap USB to RS485 (which is a two wire interface) to allow us to connect to our Raspberry Pi. Note RS485 is completely different from RS232, do not attempt to use the wrong interface. log-on to eBay and search for RS422 interfaces and you will have one in your possession within a few days. If you would like to read up about the differences between RS232, RS422 and RS485, please have a short <a href="https://www.omega.co.uk/techref/das/rs-232-422-485.html#">Google search</a> for them. RS422 and RS485 are similar, just full-duplex verses half-duplex.</p>
<p><img src="https://www.andywhittaker.com/img/RS485-Interface01.jpg" alt="RS485 Interface"></p>
<p>Link the interface up with the inverter with a length of CAT5 pair of wire. Be sure to match-up the + and - connections on either end. In my case, TX- connects to D- and TX+ connects to D+.</p>
<h2 id="modbus">Modbus</h2>
<p>A quick word on the inverter (and almost all that I have seen) is that they communicate over the <a href="https://en.wikipedia.org/wiki/Modbus">ModBus</a> Protocol. This is entirely different from whether you are connected via RS232, RS422, RS485, TCP/IP, etc.</p>
<p>They have been used for Industrial Applications since the beginning of time! You can’t talk to them via a terminal by randomly typing in commands, nothing will work. You need a ModBus (generally) RTU emulator but the good news is that there are many available to download.</p>
<h2 id="setting-up-the-raspberry-pi">Setting up the Raspberry Pi</h2>
<h3 id="emonpi">EmonPi</h3>
<p>The guys over at <a href="https://openenergymonitor.org/">openenergymonitor.org</a> have created a really easy guide to have your Raspberry Pi up and running with emoncms in no time. If you would like to use <a href="https://github.com/openenergymonitor/emonpi/wiki/emonSD-pre-built-SD-card-Download-&amp;-Change-Log#emonsd-17oct19-stable">emonpi</a> head along to their <a href="https://github.com/openenergymonitor/emonpi">GitHub repository</a> and follow the instructions.</p>
<p>Don’t forget to do the usual</p>
<pre><code>sudo apt update &amp;&amp; sudo apt full-upgrade
</code></pre>
<p>afterwards to ensure your Pi is up to date. While you are setting up, make sure you enable SSH access so that you don’t have to connect a monitor and keyboard while you set the whole thing up.</p>
<p>For SSH access I love the <a href="https://www.bitvise.com/ssh-client-download">Bitvise SSH Client</a> because it not only allows you to bring up the remote bash console but it also enables you to exchange files with your PC. This is incredibly valuable when you are developing or setting up the Pi. It also allows you to copy to other UNIX devices as well.</p>
<p>On the PC end, I think <a href="https://notepad-plus-plus.org/">NotePad++</a> is the editor to use at the moment.</p>
<h3 id="usb-interface">USB Interface</h3>
<p>Plug the USB RS485 interface into a spare USB port and reboot the Pi. SSH back in and type</p>
<pre><code>dmesg | grep tty
</code></pre>
<p>dmesg displays a copy of the Pi’s boot log and grep does a search through it for anything that matches tty. This will give you the port number that your interface is attached to (look for ttyUSB0 or similar).</p>
<p>Once you know the USB port name, you will have to update its name within the <a href="http://sofar.py">sofar.py</a> file, look for<br>
<code>instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1) # port name, slave address</code>.</p>
<p>For debugging, simply remove the comment lines (#) in front of the print statements.</p>
<h2 id="executing-the-code">Executing the Code</h2>
<p>Assuming you can see sofar,py when you type “ls”, then to run it type</p>
<pre><code>python sofar.py
</code></pre>
<p>If you get any errors, you are probably missing some python libraries which you will need to install using “pip”. Google is your friend here.</p>
<h2 id="node-red">Node-RED</h2>
<p>If your script is working, you need a way of executing it every, say, 30 seconds to log to the emonCMS database to give you pretty graphs to bore your friends with.</p>
<p><img src="http://andywhittaker.com/img/nodered.png" alt="Node-RED Timer Script"><br>
The node-RED setup actually took me the longest to complete. I was looking everywhere to find a solution and downloading libraries and all sorts. In the end it was quite easy.</p>
<p>If you would like the above flow, it’s available in the repository under flows.json.</p>
<h1 id="the-future">The Future</h1>
<p>Everything we all do is never finished. I would like to get the script to write to my Domoticz server and, in future, instruct the inverter to charge up its batteries during cheap rate electricity.</p>
<p>Oh, did I mention batteries? I have adopted PylonTech US3000 units because they seem to be the best value units on the market (2020). Although this inverter will allow you to charge from your mains supply, this is actually frowned upon (i.e. illegal) here in the UK because every time you discharge the batteries, they are being metered as energy from your SolarPV panels. If you were to charge them from the mains during the night, you will probably be visited by some very angry men who will be in no mood for humour…</p>
<p>An alternative that anyone could do is to use the Sofa ME3000 inverter. This is just wired up to your mains supply and is able to charge the batteries when it sees your system trying to export to the grid. By clever voltage monitoring, if export drops, it then discharges the batteries into your home. From what I can see, the HYD6000 and ME3000 have the same ModBus communication protocol. The only downside is that these inverters are limited to a maximum discharge load of 3000W.</p>

