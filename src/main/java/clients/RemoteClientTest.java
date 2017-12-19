/*
 * Copyright (C) 2014 Pablo Campillo-Sanchez <pabcampi@ucm.es>
 *
 * This software has been developed as part of the 
 * SociAAL project directed by Jorge J. Gomez Sanz
 * (http://grasia.fdi.ucm.es/sociaal)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package clients;

import com.google.common.net.InetAddresses;
import com.jme3.app.Application;
import com.jme3.app.SimpleApplication;
import com.jme3.app.state.AbstractAppState;
import com.jme3.app.state.AppStateManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.net.Inet4Address;
import java.net.Inet6Address;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Date;
import java.util.logging.Logger;

import javax.swing.SwingUtilities;

import org.omg.CORBA.portable.InputStream;

import phat.app.PHATApplication;
import phat.app.PHATInitAppListener;
import phat.body.BodiesAppState;
import phat.body.commands.FallDownCommand;
import phat.body.commands.RandomWalkingCommand;
import phat.body.commands.SetBodyInCoordenatesCommand;
import phat.body.commands.SetCameraToBodyCommand;
import phat.body.commands.SetRigidArmCommand;
import phat.body.commands.SetSpeedDisplacemenetCommand;
import phat.body.commands.SetStoopedBodyCommand;
import phat.body.commands.StandUpCommand;
import phat.body.commands.TremblingHandCommand;
import phat.body.commands.TremblingHeadCommand;
import phat.body.commands.TripOverCommand;
import phat.commands.PHATCommand;
import phat.devices.DevicesAppState;
import phat.devices.commands.CreateAccelerometerSensorCommand;
import phat.devices.commands.CreateSmartphoneCommand;
import phat.devices.commands.SetDeviceOnPartOfBodyCommand;
import phat.mobile.servicemanager.server.ServiceManagerServer;
import phat.mobile.servicemanager.services.Service;
import phat.sensors.accelerometer.AccelerationData;
import phat.sensors.accelerometer.AccelerometerControl;
import phat.sensors.accelerometer.XYAccelerationsChart;
import phat.server.ServerAppState;
import phat.server.commands.ActivateAccelerometerServerCommand;
import phat.server.commands.DisplayAVDScreenCommand;
import phat.server.commands.PHATServerCommand;
import phat.server.commands.SetAndroidEmulatorCommand;
import phat.server.commands.StartActivityCommand;
import phat.structures.houses.TestHouse;
import phat.util.Debug;
import phat.util.SpatialFactory;
import phat.world.WorldAppState;
import server.XYShiftingAccelerationsChart;
import sim.android.hardware.service.SimSensorEvent;

/**
 *
 * @author pablo
 */
public class RemoteClientTest  {

	private static final Logger logger = Logger.getLogger(TestHouse.class.getName());


	public static void main(String[] args) {
	
		
			launchRemoteXYChart("remote","127.0.0.1",60000);
		
		
	}


	private static void launchRemoteXYChart(final String title, final String host, final int port) {

				
				new Thread(){
					public void run(){
						Socket s;
						try {
							final XYShiftingAccelerationsChart chart = new XYShiftingAccelerationsChart(title, "Chest accelerations", "m/s2", "x,y,z");
							chart.showWindow();
							s = new Socket(host,port);
							
							BufferedReader is=new BufferedReader(new InputStreamReader(s.getInputStream()));

							String objRead=null;

							Long lastRead=new Date().getTime();
							do {
								
								objRead=is.readLine();
								final long interval=new Date().getTime()-lastRead;
								lastRead=new Date().getTime();
								
								if (objRead!=null && !objRead.isEmpty()){
									final SimSensorEvent sse=SimSensorEvent.fromString(objRead);
									if (sse!=null){										
										SwingUtilities.invokeLater(new Runnable(){
											public void run(){											
												AccelerationData ad = new AccelerationData(
														interval,sse.getValues()[0],sse.getValues()[1],sse.getValues()[2]);
												chart.update(null,ad);
												chart.repaint();
											}
										});
									}
								};
							} while (objRead!=null);
						} catch (UnknownHostException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} catch (IOException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						} 
						
					}
				}.start();
				
			

	}
}