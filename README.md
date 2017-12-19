Demo of a dummy producing sensory input of an accelerometer in the chest and in the left hand.

To run the demo

	mvn clean compile
	mvn exec:java -Dexec.mainClass=server.RemoteAccelerometerParkinsonTest

To run an external listingin client (while running the demo)

	mvn exec:java -Dexec.mainClass=clients.RemoteClientTest

In case of running into memory problems

	export MAVEN_OPTS="-Xmx512m -XX:MaxPermSize=128m"

And then run the previous command


