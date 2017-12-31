/*
 * Copyright (C) 2014 Pablo Campillo-Sanchez <pabcampi@ucm.es>
 * Copyright (C) 2017 Jorge Gomez Sanz <jjgomez@ucm.es>
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
package server;

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
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Date;
import java.util.Vector;
import java.util.logging.Logger;

import javax.swing.SwingUtilities;

import org.omg.CORBA.portable.InputStream;

import phat.app.PHATApplication;
import phat.app.PHATInitAppListener;
import phat.body.BodiesAppState;
import phat.body.commands.FallDownCommand;
import phat.body.commands.GoToCommand;
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
import phat.body.control.navigation.AutonomousControlListener;
import phat.commands.PHATCommand;
import phat.commands.PHATCommand.State;
import phat.commands.PHATCommandListener;
import phat.devices.DevicesAppState;
import phat.devices.commands.CreateAccelerometerSensorCommand;
import phat.devices.commands.CreateSmartphoneCommand;
import phat.devices.commands.SetDeviceOnPartOfBodyCommand;
import phat.mobile.servicemanager.server.ServiceManagerServer;
import phat.mobile.servicemanager.services.Service;
import phat.sensors.accelerometer.AccelerationData;
import phat.sensors.accelerometer.AccelerometerControl;
import server.XYShiftingAccelerationsChart;
import phat.server.PHATServerManager;
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
import sim.android.hardware.service.SimSensorEvent;

class GotoPath extends PHATCommand implements PHATCommandListener {
	Vector<GoToCommand> gotos = new Vector<GoToCommand>();
	Vector<GoToCommand> gotostorun = null;
	BodiesAppState body;
	boolean loop = false;
	boolean initiated = false;
	private Vector3f[] positions;
	private int resumingIndex;
	private String commandid;
	private GoToCommand currentCommand;

	public GotoPath(BodiesAppState body, String id, Vector3f[] positions, boolean loop) {
		this.body = body;
		gotos = createGoTos(id, positions);
		gotostorun = new Vector<GoToCommand>(gotos);
		this.positions = positions;
		this.commandid = id;
		this.loop = loop;
	}

	Vector<GoToCommand> createGoTos(String id, Vector3f[] positions) {
		Vector<GoToCommand> gotos = new Vector<GoToCommand>();
		for (Vector3f pos : positions) {
			final Vector3f finalPos = pos;
			GoToCommand gt = new GoToCommand(id, new phat.util.Lazy<Vector3f>() {
				@Override
				public Vector3f getLazy() {
					return finalPos;
				}
			}, this);
			gotos.add(gt);
		}
		return gotos;
	}

	@Override
	public void runCommand(Application app) {
		if (!initiated) {
			// launches the first command. Afterwards, this is useless.
			runNextGoto();
			initiated = true;
		}
	}

	public void resume() {
		if (currentCommand != null && currentCommand.getState().equals(State.Interrupted)) {
			gotostorun = createGoTos(commandid, positions);
			// recreates the structure to recreate the last interrupted command
			for (int k = 0; k < resumingIndex; k++)
				gotostorun.removeElementAt(0);
			resumingIndex=resumingIndex-1;// situation previous to the last command
			runNextGoto();
		}
	}

	@Override
	public void interruptCommand(Application app) {
		if (currentCommand != null)
			currentCommand.interruptCommand(app);
		this.setState(State.Interrupted);
	}

	private void runNextGoto() {
		currentCommand = gotostorun.firstElement();
		resumingIndex++;

		gotostorun.removeElementAt(0);
		if (gotostorun.isEmpty() && loop) {
			gotostorun = createGoTos(commandid, positions);
			body.runCommand(currentCommand);
			resumingIndex = 0;
		} else if (!gotostorun.isEmpty()) {
			body.runCommand(currentCommand);
		} else
			this.setState(State.Success);

	}

	@Override
	public void commandStateChanged(PHATCommand command) {
		if (command.getState().equals(State.Success)) {
			runNextGoto();
		}

	}
}

/**
 *
 * @author pablo
 */
public class RemoteAccelerometerParkinsonTest implements PHATInitAppListener {

	private static final Logger logger = Logger.getLogger(TestHouse.class.getName());
	BodiesAppState bodiesAppState;
	ServerAppState serverAppState;
	DevicesAppState devicesAppState;
	WorldAppState worldAppState;

	public static void main(String[] args) {
		RemoteAccelerometerParkinsonTest test = new RemoteAccelerometerParkinsonTest();
		PHATApplication phat = new PHATApplication(test);
		phat.setDisplayFps(true);
		phat.setDisplayStatView(false);
		AppSettings settings = new AppSettings(true);
		settings.setTitle("PHAT");
		settings.setWidth(640);
		settings.setHeight(480);
		phat.setSettings(settings);
		phat.start();
	}

	@Override
	public void init(SimpleApplication app) {
		SpatialFactory.init(app.getAssetManager(), app.getRootNode());

		AppStateManager stateManager = app.getStateManager();

		app.getFlyByCamera().setMoveSpeed(10f);

		app.getCamera().setLocation(new Vector3f(0.2599395f, 2.7232018f, 3.373138f).mult(3f));
		app.getCamera().setRotation(new Quaternion(-0.0035931943f, 0.9672268f, -0.25351822f, -0.013704466f));

		BulletAppState bulletAppState = new BulletAppState();
		bulletAppState.setThreadingType(BulletAppState.ThreadingType.PARALLEL);
		stateManager.attach(bulletAppState);
		bulletAppState.getPhysicsSpace().setAccuracy(1 / 60f);
		// bulletAppState.setDebugEnabled(true);

		worldAppState = new WorldAppState();
		worldAppState.setLandType(WorldAppState.LandType.Grass);
		app.getStateManager().attach(worldAppState);
		worldAppState.setCalendar(2013, 1, 1, 12, 0, 0);

		//Debug.enableDebugGrid(10, app.getAssetManager(), app.getRootNode());
		bodiesAppState = new BodiesAppState();
		stateManager.attach(bodiesAppState);

		bodiesAppState.createBody(BodiesAppState.BodyType.ElderLP, "Patient");
		bodiesAppState.runCommand(new SetBodyInCoordenatesCommand("Patient", new Vector3f(2.0f, 0f, 2f)));
		// bodiesAppState.runCommand(new RandomWalkingCommand("Patient", true));
		final GotoPath gotopathCommand = new GotoPath(
				bodiesAppState, "Patient", new Vector3f[] { new Vector3f(2.0f, 0f, 2f),
						new Vector3f(4f, 0, 4f), new Vector3f(4f, 0.0f, 2f), new Vector3f(2, 0, 2) },
				true);
		bodiesAppState
				.runCommand(gotopathCommand
						);

		bodiesAppState.runCommand(new TremblingHandCommand("Patient", true, true));
		TremblingHeadCommand thc = new TremblingHeadCommand("Patient", true);
		thc.setAngular(FastMath.HALF_PI);
		bodiesAppState.runCommand(thc);

		bodiesAppState.runCommand(new SetSpeedDisplacemenetCommand("Patient", 0.5f));
		// bodiesAppState.runCommand(new SetRigidArmCommand("Patient", true, true));
		// bodiesAppState.runCommand(new SetRigidArmCommand("Patient", true, false));
		bodiesAppState.runCommand(new SetStoopedBodyCommand("Patient", true));

		SetCameraToBodyCommand camCommand = new SetCameraToBodyCommand("Patient");
		camCommand.setDistance(3);
		camCommand.setFront(false);
		bodiesAppState.runCommand(camCommand);

		devicesAppState = new DevicesAppState();
		stateManager.attach(devicesAppState);

		devicesAppState.runCommand(new CreateAccelerometerSensorCommand("sensor1"));
		devicesAppState.runCommand(
				new SetDeviceOnPartOfBodyCommand("Patient", "sensor1", SetDeviceOnPartOfBodyCommand.PartOfBody.Chest));

		devicesAppState.runCommand(new CreateAccelerometerSensorCommand("sensor2"));
		devicesAppState.runCommand(new SetDeviceOnPartOfBodyCommand("Patient", "sensor2",
				SetDeviceOnPartOfBodyCommand.PartOfBody.LeftHand));

		serverAppState = new ServerAppState();
		stateManager.attach(serverAppState);

		serverAppState.runCommand(new ActivateAccelerometerServerCommand("accel", "sensor1"));
		serverAppState.runCommand(new ActivateAccelerometerServerCommand("accel", "sensor2"));

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		
		  new Thread() { public void run() {
		  launchRemoteXYChart(PHATServerManager.getAddress(),"Remote Chest Sensor"
		  ,"sensor1");
		  launchRemoteXYChart(PHATServerManager.getAddress(),"Remote Left Hand"
		  ,"sensor2");
		  
		  } }.start();
		 

		stateManager.attach(new AbstractAppState() {
			PHATApplication app;

			@Override
			public void initialize(AppStateManager asm, Application aplctn) {
				app = (PHATApplication) aplctn;

			}

			boolean standUp = false;
			boolean washingHands = false;
			boolean havingShower = false;
			float cont = 0f;
			boolean fall = false;
			float timeToFall = 10f;
			boolean init = false;

			@Override
			public void update(float f) {
				if (!init) {
					
					  AccelerometerControl ac =
					  devicesAppState.getDevice("sensor1").getControl(AccelerometerControl.class);
					  ac.setMode(AccelerometerControl.AMode.GRAVITY_MODE);
					 /* XYShiftingAccelerationsChart chart = new
					  XYShiftingAccelerationsChart("Chart - Acc.", "Local acceleration chest",
					  "m/s2", "x,y,z"); ac.add(chart); chart.showWindow(); init = true;*/
					  init = true;
					 

				}
				cont += f;
				if (cont > timeToFall && !fall) {
					bodiesAppState.runCommand(new FallDownCommand("Patient"));
					fall = true;
				} else if (fall && cont > timeToFall + 6) {
					PHATCommand standUp = new StandUpCommand("Patient");
					standUp.setListener(new PHATCommandListener() {

						@Override
						public void commandStateChanged(PHATCommand command) {
							if (command!=null && command.getState().equals(State.Success)) {
								gotopathCommand.resume();
							}
							
						}
						
					});
					bodiesAppState.runCommand(standUp);
					fall = false;
					cont = 0;
				}
			}
		});
	}

	public static void launchRemoteXYChart(final InetAddress host, final String title, final String sensor) {
		Service taccelService = ServiceManagerServer.getInstance().getServiceManager().getService("accel", sensor);
		while (taccelService == null) {
			// not ready
			try {
				Thread.currentThread().sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			taccelService = ServiceManagerServer.getInstance().getServiceManager().getService("accel", sensor);
		}
		final Service accelService = taccelService;
		new Thread() {
			public void run() {
				Socket s = null;
				try {
					final XYShiftingAccelerationsChart chart = new XYShiftingAccelerationsChart(title,
							"Remote " + sensor + ":" + title + " accelerations", "m/s2", "x,y,z");
					chart.showWindow();
					for (int k = 0; k < 5 && s == null; k++)
						try {
							s = new Socket(host, accelService.getPort());
						} catch (IOException e) {
							e.printStackTrace();
							try {
								Thread.currentThread().sleep(500);
							} catch (InterruptedException e1) {
								e1.printStackTrace();
							}

						}

					BufferedReader is = new BufferedReader(new InputStreamReader(s.getInputStream()));

					String objRead = null;
					Long lastRead = new Date().getTime();
					do {

						objRead = is.readLine();
						final long interval = new Date().getTime() - lastRead;
						lastRead = new Date().getTime();
						if (objRead != null && !objRead.isEmpty()) {
							final SimSensorEvent sse = SimSensorEvent.fromString(objRead);
							if (sse != null) {
								SwingUtilities.invokeLater(new Runnable() {
									public void run() {
										AccelerationData ad = new AccelerationData(interval, sse.getValues()[0],
												sse.getValues()[1], sse.getValues()[2]);
										chart.update(null, ad);
										chart.repaint();
									}
								});
							}
						}
						;
					} while (objRead != null);
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
