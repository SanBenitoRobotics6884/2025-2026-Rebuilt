package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class LIDAR {
	private I2C i2c;
	private byte[] distance;
	private java.util.Timer updater;
	
	private final int LIDAR_ADDR = 0x62;
	private final int LIDAR_CONFIG_REGISTER = 0x00;
	private final int LIDAR_DISTANCE_REGISTER = 0x8f;
	
	public LIDAR(Port port) {
		i2c = new I2C(port, LIDAR_ADDR);
		
		distance = new byte[2];
		
		updater = new java.util.Timer();
	}
}
