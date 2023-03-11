package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * BNO055 IMU for the FIRST Robotics Competition.
 * References throughout the code are to the following sensor documentation:
 *   http://git.io/vuOl1
 * 
 * To use the sensor, wire up to it over I2C on the roboRIO.
 * Creating an instance of this class will cause communications with the sensor
 *   to being.All communications with the sensor occur in a separate thread
 *   from your robot code to avoid blocking the main robot program execution.
 * 
 *  Example:
 *    private static BNO055 imu;
 *    
 *    public Robot() {
 *        imu = BNO055(BNO055.opmode_t.OPERATION_MODE_IMUPLUS,
 *        		BNO055.vector_type_t.VECTOR_EULER);
 *    }
 * 
 * You can check the status of the sensor by using the following methods:
 *   isSensorPresent(); //Checks if the code can talk to the sensor over I2C
 *                      // If this returns false, check your wiring.
 *   isInitialized(); //Checks if the sensor initialization has completed.
 *                    // Initialization takes about 3 seconds. You won't get
 *                    // position data back from the sensor until its init'd. 
 *   isCalibrated(); //The BNO055 will return position data after its init'd,
 *   				 // but the position data may be inaccurate until all
 *                   // required sensors report they are calibrated. Some
 *                   // Calibration sequences require you to move the BNO055
 *                   // around. See the method comments for more info.
 *
 * Once the sensor calibration is complete , you can get position data by
 *   by using the getVector() method. See this method definiton for usage info.
 * 
 * This code was originally ported from arduino source developed by Adafruit.
 * See the original comment header below.
 * 
 * @author james@team2168.org
 *
 *
 *ORIGINAL ADAFRUIT HEADER - https://github.com/adafruit/Adafruit_BNO055/
 *=======================================================================
 *This is a library for the BNO055 orientation sensor
 *
 *Designed specifically to work with the Adafruit BNO055 Breakout.
 *
 *Pick one up today in the adafruit shop!
 *------> http://www.adafruit.com/products
 *
 *These sensors use I2C to communicate, 2 pins are required to interface.
 *
 *Adafruit invests time and resources providing this open source code,
 *please support Adafruit and open-source hardware by purchasing products
 *from Adafruit!
 *
 *Written by KTOWN for Adafruit Industries.
 *
 *MIT license, all text above must be included in any redistribution
 *
 */
public class BNO055 {
	//Tread variables
	private java.util.Timer executor;
	private static final long THREAD_PERIOD = 20; //ms - max poll rate on sensor.
	
	public static final byte BNO055_ADDRESS_A = 0x28;
	public static final byte BNO055_ADDRESS_B = 0x29;
	public static final int BNO055_ID = 0xA0;
	
	private I2C imu;
	private String name; // used to label logs and prints
	private int _mode;
	private opmode_t requestedMode; //user requested mode of operation.
	private vector_type_t requestedVectorType;
	private BNO055OffsetData offsets;  // Offsets to write during initialization.
	
	//State machine variables
	private volatile int state = 0;
	private volatile boolean sensorPresent = false;
	private volatile boolean initialized = false;
	private volatile byte[] positionVector = new byte[6];
	private volatile long turns = 0;
	private volatile double[] xyz = new double[3];

	private double headingOffset = 0.0; //degrees

	public class SystemStatus {
		public int system_status;
		public int self_test_result;
		public int system_error;
	}

	public enum reg_t {
		/* Page id register definition */
		BNO055_PAGE_ID_ADDR                                     (0X07),

		/* PAGE0 REGISTER DEFINITION START*/
		BNO055_CHIP_ID_ADDR                                     (0x00),
		BNO055_ACCEL_REV_ID_ADDR                                (0x01),
		BNO055_MAG_REV_ID_ADDR                                  (0x02),
		BNO055_GYRO_REV_ID_ADDR                                 (0x03),
		BNO055_SW_REV_ID_LSB_ADDR                               (0x04),
		BNO055_SW_REV_ID_MSB_ADDR                               (0x05),
		BNO055_BL_REV_ID_ADDR                                   (0X06),

		/* Accel data register */
		BNO055_ACCEL_DATA_X_LSB_ADDR                            (0X08),
		BNO055_ACCEL_DATA_X_MSB_ADDR                            (0X09),
		BNO055_ACCEL_DATA_Y_LSB_ADDR                            (0X0A),
		BNO055_ACCEL_DATA_Y_MSB_ADDR                            (0X0B),
		BNO055_ACCEL_DATA_Z_LSB_ADDR                            (0X0C),
		BNO055_ACCEL_DATA_Z_MSB_ADDR                            (0X0D),

		/* Mag data register */
		BNO055_MAG_DATA_X_LSB_ADDR                              (0X0E),
		BNO055_MAG_DATA_X_MSB_ADDR                              (0X0F),
		BNO055_MAG_DATA_Y_LSB_ADDR                              (0X10),
		BNO055_MAG_DATA_Y_MSB_ADDR                              (0X11),
		BNO055_MAG_DATA_Z_LSB_ADDR                              (0X12),
		BNO055_MAG_DATA_Z_MSB_ADDR                              (0X13),

		/* Gyro data registers */
		BNO055_GYRO_DATA_X_LSB_ADDR                             (0X14),
		BNO055_GYRO_DATA_X_MSB_ADDR                             (0X15),
		BNO055_GYRO_DATA_Y_LSB_ADDR                             (0X16),
		BNO055_GYRO_DATA_Y_MSB_ADDR                             (0X17),
		BNO055_GYRO_DATA_Z_LSB_ADDR                             (0X18),
		BNO055_GYRO_DATA_Z_MSB_ADDR                             (0X19),

		/* Euler data registers */
		BNO055_EULER_H_LSB_ADDR                                 (0X1A),
		BNO055_EULER_H_MSB_ADDR                                 (0X1B),
		BNO055_EULER_R_LSB_ADDR                                 (0X1C),
		BNO055_EULER_R_MSB_ADDR                                 (0X1D),
		BNO055_EULER_P_LSB_ADDR                                 (0X1E),
		BNO055_EULER_P_MSB_ADDR                                 (0X1F),

		/* Quaternion data registers */
		BNO055_QUATERNION_DATA_W_LSB_ADDR                       (0X20),
		BNO055_QUATERNION_DATA_W_MSB_ADDR                       (0X21),
		BNO055_QUATERNION_DATA_X_LSB_ADDR                       (0X22),
		BNO055_QUATERNION_DATA_X_MSB_ADDR                       (0X23),
		BNO055_QUATERNION_DATA_Y_LSB_ADDR                       (0X24),
		BNO055_QUATERNION_DATA_Y_MSB_ADDR                       (0X25),
		BNO055_QUATERNION_DATA_Z_LSB_ADDR                       (0X26),
		BNO055_QUATERNION_DATA_Z_MSB_ADDR                       (0X27),

		/* Linear acceleration data registers */
		BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     (0X28),
		BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     (0X29),
		BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     (0X2A),
		BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     (0X2B),
		BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     (0X2C),
		BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     (0X2D),

		/* Gravity data registers */
		BNO055_GRAVITY_DATA_X_LSB_ADDR                          (0X2E),
		BNO055_GRAVITY_DATA_X_MSB_ADDR                          (0X2F),
		BNO055_GRAVITY_DATA_Y_LSB_ADDR                          (0X30),
		BNO055_GRAVITY_DATA_Y_MSB_ADDR                          (0X31),
		BNO055_GRAVITY_DATA_Z_LSB_ADDR                          (0X32),
		BNO055_GRAVITY_DATA_Z_MSB_ADDR                          (0X33),

		/* Temperature data register */
		BNO055_TEMP_ADDR                                        (0X34),

		/* Status registers */
		BNO055_CALIB_STAT_ADDR                                  (0X35),
		BNO055_SELFTEST_RESULT_ADDR                             (0X36),
		BNO055_INTR_STAT_ADDR                                   (0X37),

		BNO055_SYS_CLK_STAT_ADDR                                (0X38),
		BNO055_SYS_STAT_ADDR                                    (0X39),
		BNO055_SYS_ERR_ADDR                                     (0X3A),

		/* Unit selection register */
		BNO055_UNIT_SEL_ADDR                                    (0X3B),
		BNO055_DATA_SELECT_ADDR                                 (0X3C),

		/* Mode registers */
		BNO055_OPR_MODE_ADDR                                    (0X3D),
		BNO055_PWR_MODE_ADDR                                    (0X3E),

		BNO055_SYS_TRIGGER_ADDR                                 (0X3F),
		BNO055_TEMP_SOURCE_ADDR                                 (0X40),

		/* Axis remap registers */
		BNO055_AXIS_MAP_CONFIG_ADDR                             (0X41),
		BNO055_AXIS_MAP_SIGN_ADDR                               (0X42),

		/* SIC registers */
		BNO055_SIC_MATRIX_0_LSB_ADDR                            (0X43),
		BNO055_SIC_MATRIX_0_MSB_ADDR                            (0X44),
		BNO055_SIC_MATRIX_1_LSB_ADDR                            (0X45),
		BNO055_SIC_MATRIX_1_MSB_ADDR                            (0X46),
		BNO055_SIC_MATRIX_2_LSB_ADDR                            (0X47),
		BNO055_SIC_MATRIX_2_MSB_ADDR                            (0X48),
		BNO055_SIC_MATRIX_3_LSB_ADDR                            (0X49),
		BNO055_SIC_MATRIX_3_MSB_ADDR                            (0X4A),
		BNO055_SIC_MATRIX_4_LSB_ADDR                            (0X4B),
		BNO055_SIC_MATRIX_4_MSB_ADDR                            (0X4C),
		BNO055_SIC_MATRIX_5_LSB_ADDR                            (0X4D),
		BNO055_SIC_MATRIX_5_MSB_ADDR                            (0X4E),
		BNO055_SIC_MATRIX_6_LSB_ADDR                            (0X4F),
		BNO055_SIC_MATRIX_6_MSB_ADDR                            (0X50),
		BNO055_SIC_MATRIX_7_LSB_ADDR                            (0X51),
		BNO055_SIC_MATRIX_7_MSB_ADDR                            (0X52),
		BNO055_SIC_MATRIX_8_LSB_ADDR                            (0X53),
		BNO055_SIC_MATRIX_8_MSB_ADDR                            (0X54),

		/* Accelerometer Offset registers */
		ACCEL_OFFSET_X_LSB_ADDR                                 (0X55),
		ACCEL_OFFSET_X_MSB_ADDR                                 (0X56),
		ACCEL_OFFSET_Y_LSB_ADDR                                 (0X57),
		ACCEL_OFFSET_Y_MSB_ADDR                                 (0X58),
		ACCEL_OFFSET_Z_LSB_ADDR                                 (0X59),
		ACCEL_OFFSET_Z_MSB_ADDR                                 (0X5A),

		/* Magnetometer Offset registers */
		MAG_OFFSET_X_LSB_ADDR                                   (0X5B),
		MAG_OFFSET_X_MSB_ADDR                                   (0X5C),
		MAG_OFFSET_Y_LSB_ADDR                                   (0X5D),
		MAG_OFFSET_Y_MSB_ADDR                                   (0X5E),
		MAG_OFFSET_Z_LSB_ADDR                                   (0X5F),
		MAG_OFFSET_Z_MSB_ADDR                                   (0X60),

		/* Gyroscope Offset register s*/
		GYRO_OFFSET_X_LSB_ADDR                                  (0X61),
		GYRO_OFFSET_X_MSB_ADDR                                  (0X62),
		GYRO_OFFSET_Y_LSB_ADDR                                  (0X63),
		GYRO_OFFSET_Y_MSB_ADDR                                  (0X64),
		GYRO_OFFSET_Z_LSB_ADDR                                  (0X65),
		GYRO_OFFSET_Z_MSB_ADDR                                  (0X66),

		/* Radius registers */
		ACCEL_RADIUS_LSB_ADDR                                   (0X67),
		ACCEL_RADIUS_MSB_ADDR                                   (0X68),
		MAG_RADIUS_LSB_ADDR                                     (0X69),
		MAG_RADIUS_MSB_ADDR                                     (0X6A);

		private final int val;

		reg_t(int val) {
			this.val = val;
		}

		public int getVal() {
			return val;
		}
	};

	public enum powermode_t	{
		POWER_MODE_NORMAL                                       (0X00),
		POWER_MODE_LOWPOWER                                     (0X01),
		POWER_MODE_SUSPEND                                      (0X02);

		private final int val;

		powermode_t(int val) {
			this.val = val;
		}

		public int getVal() {
			return val;
		}
	};

	public enum opmode_t {
		/* Operation mode settings*/
		OPERATION_MODE_CONFIG                                   (0X00),
		OPERATION_MODE_ACCONLY                                  (0X01),
		OPERATION_MODE_MAGONLY                                  (0X02),
		OPERATION_MODE_GYRONLY                                  (0X03),
		OPERATION_MODE_ACCMAG                                   (0X04),
		OPERATION_MODE_ACCGYRO                                  (0X05),
		OPERATION_MODE_MAGGYRO                                  (0X06),
		OPERATION_MODE_AMG                                      (0X07),
		OPERATION_MODE_IMUPLUS                                  (0X08),
		OPERATION_MODE_COMPASS                                  (0X09),
		OPERATION_MODE_M4G                                      (0X0A),
		OPERATION_MODE_NDOF_FMC_OFF                             (0X0B),
		OPERATION_MODE_NDOF                                     (0X0C);

		private final int val;

		opmode_t(int val) {
			this.val = val;
		}

		public int getVal() {
			return val;
		}

		public static opmode_t fromByte(byte mode) {
			for (opmode_t enumVal : values()) {
				if (enumVal.getVal() == mode) {
					return enumVal;
				}
			}
			return null;
		}
	}

	public class RevInfo {
		public byte accel_rev;
		public byte mag_rev;
		public byte gyro_rev;
		public short sw_rev;
		public byte bl_rev;
	}

	public class CalData {
		public byte sys;
		public byte gyro;
		public byte accel;
		public byte mag;
	}

	public enum vector_type_t {
		VECTOR_ACCELEROMETER (reg_t.BNO055_ACCEL_DATA_X_LSB_ADDR.getVal()),
		VECTOR_MAGNETOMETER  (reg_t.BNO055_MAG_DATA_X_LSB_ADDR.getVal()),
		VECTOR_GYROSCOPE     (reg_t.BNO055_GYRO_DATA_X_LSB_ADDR.getVal()),
		VECTOR_EULER         (reg_t.BNO055_EULER_H_LSB_ADDR.getVal()),
		VECTOR_LINEARACCEL   (reg_t.BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR.getVal()),
		VECTOR_GRAVITY       (reg_t.BNO055_GRAVITY_DATA_X_LSB_ADDR.getVal());

		private final int val;

		vector_type_t(int val) {
			this.val = val;
		}

		public int getVal() {
			return val;
		}
	};
	
	/**
	 * Instantiates a new BNO055 class.
	 *
	 * @param mode the desired operation mode of the device
	 * @param vectorType the desired output vector type
	 */
	public BNO055(opmode_t mode, vector_type_t vectorType) {
		this(I2C.Port.kMXP, BNO055_ADDRESS_A, "BNO055", mode, vectorType, new BNO055OffsetData());
	}
	
	/**
	 * Instantiates a new BNO055 class.
	 *
	 * @param port the physical port the sensor is plugged into on the roboRio
	 * @param address the address the sensor is at (0x28 or 0x29)
	 * @param nametag a string to identify the BNO055 in logs and the SmartDashboard
	 * @param mode the desired operation mode of the device
	 * @param vectorType the desired output vector type
	 * @param offsetData calibration offsets read from the device previously
	 */
	public BNO055(I2C.Port port, byte address, String nametag, opmode_t mode, vector_type_t vectorType, BNO055OffsetData offsetData) {
		imu = new I2C(port, address);
		name = nametag;
		executor = new java.util.Timer();
		requestedMode = mode;
		requestedVectorType = vectorType;
		offsets = offsetData;
		executor.schedule(new BNO055UpdateTask(this), 0L, THREAD_PERIOD);
	}


	/**
	 * Called periodically. Communicates with the sensor, and checks its state. 
	 */
	private void update() {
		if(!initialized) {			
			//Step through process of initializing the sensor in a non-
			//  blocking manner. This sequence of events follows the process
			//  defined in the original adafruit source as closely as possible.
			//  NOTE: This has been modified to more closely match the process from the bno055 c driver from Bosch.
			switch(state) {
			case 0:
				//Wait for the sensor to be present
				if((0xFF & read8(reg_t.BNO055_CHIP_ID_ADDR)) != BNO055_ID) {
					//Sensor not present, keep trying
					sensorPresent = false;
				} else {
					//Sensor present, go to next state
					sensorPresent = true;
					state++;
					println("sensor detected, beginning initialization.");
				}
				break;
			case 1:
				// Reset
				println("resetting.");
				write8(reg_t.BNO055_SYS_TRIGGER_ADDR, (byte) 0x20);
				state++;
				break;
			case 2:
				//Wait for the sensor to be present
				if((0xFF & read8(reg_t.BNO055_CHIP_ID_ADDR)) == BNO055_ID) {
					//Sensor present, go to next state
					println("back online.");
					state++;
				}
				break;
			case 3:
				//Use external crystal - 32.768 kHz
				println("clearing page.");
				write8(reg_t.BNO055_PAGE_ID_ADDR, (byte) 0x00);
				state++;
				break;
			case 4:
				if (offsets != null) {
					writeOffsets(offsets);
				}
				state++;
				break;
			case 5:
				//Set operating mode to mode requested at instantiation
				setMode(requestedMode);
				state++;
				break;
			case 6:
				//Set operating mode to mode requested at instantiation
				printSystemStatus();
				state++;
				break;
			case 7:
				println("initialized.");
				initialized = true;
				break;
			default:
				//Should never get here - Fail safe
				initialized = false;
			}
		} else {
			//Sensor is initialized, periodically query position data
			calculateVector();
		}
	}

	/**
	 * Query the sensor for position data.
	 */
	private void calculateVector() {
		double[] pos = new double[3];
		if (readLen(requestedVectorType.getVal(), positionVector)) {
			// Read vector data (6 bytes)
			short x = 0, y = 0, z = 0;
			double headingDiff = 0.0;
			x = (short)((positionVector[0] & 0xFF)
					| ((positionVector[1] << 8) & 0xFF00));
			y = (short)((positionVector[2] & 0xFF)
					| ((positionVector[3] << 8) & 0xFF00));
			z = (short)((positionVector[4] & 0xFF)
					| ((positionVector[5] << 8) & 0xFF00));
			
			/* Convert the value to an appropriate range (section 3.6.4) */
			/* and assign the value to the Vector type */
			switch(requestedVectorType) {
			case VECTOR_MAGNETOMETER:
				/* 1uT = 16 LSB */
				pos[0] = ((double)x)/16.0;
				pos[1] = ((double)y)/16.0;
				pos[2] = ((double)z)/16.0;
				break;
			case VECTOR_GYROSCOPE:
				/* 1rps = 900 LSB */
				pos[0] = ((double)x)/900.0;
				pos[1] = ((double)y)/900.0;
				pos[2] = ((double)z)/900.0;
				break;
			case VECTOR_EULER:
				/* 1 degree = 16 LSB */
				pos[0] = ((double)x)/16.0;
				pos[1] = ((double)y)/16.0;
				pos[2] = ((double)z)/16.0;
				break;
			case VECTOR_ACCELEROMETER:
			case VECTOR_LINEARACCEL:
			case VECTOR_GRAVITY:
				/* 1m/s^2 = 100 LSB */
				pos[0] = ((double)x)/100.0;
				pos[1] = ((double)y)/100.0;
				pos[2] = ((double)z)/100.0;
				break;
			}
			
			//calculate turns
			headingDiff = xyz[0] - pos[0];
			if(Math.abs(headingDiff) >= 350) {
				//We've traveled past the zero heading position
				if(headingDiff > 0) {
					turns++;
				} else {
					turns--;
				}
			}
			
			//Update position vectors
			xyz = pos;
		} else {
			// Read error, assume device needs to be reset
			println("error reading data. Resetting device.");
			initialized = false;
			xyz = pos;
			turns = 0;
			state = 0;
		}

	}

	public opmode_t getMode() {
		return opmode_t.fromByte(read8(reg_t.BNO055_OPR_MODE_ADDR));
	}
	
	/**
	 * Puts the chip in the specified operating mode
	 * @param mode
	 */
	public void setMode(opmode_t mode) {
		opmode_t currentMode = getMode();
		if (currentMode == mode) {
			println("already in mode " + mode.name());
			return;
		}
		if (currentMode != opmode_t.OPERATION_MODE_CONFIG) {
			// must switch to config mode before desired mode
			println("switching to config mode.");
			write8(reg_t.BNO055_OPR_MODE_ADDR, (byte) opmode_t.OPERATION_MODE_CONFIG.getVal());
			currentMode = opmode_t.OPERATION_MODE_CONFIG;
			Timer.delay(0.020);
		}
		if (currentMode != mode) {
			// If the desired mode was config, this will be skipped
			println(String.format("switching to mode %s", mode.name()));
			write8(reg_t.BNO055_OPR_MODE_ADDR, (byte) mode.getVal());
			Timer.delay(0.600);
		}
	}

	/**
	 * Gets the latest system status info
	 * @return
	 */
	public SystemStatus getSystemStatus() {
		SystemStatus status = new SystemStatus();

		write8(reg_t.BNO055_PAGE_ID_ADDR, (byte) 0x00);

		/* System Status (see section 4.3.58)
		   ---------------------------------
		   0 = Idle
		   1 = System Error
		   2 = Initializing Peripherals
		   3 = System Initalization
		   4 = Executing Self-Test
		   5 = Sensor fusion algorithm running
		   6 = System running without fusion algorithms */

		status.system_status = read8(reg_t.BNO055_SYS_STAT_ADDR);

		/* Self Test Results (see section )
		   --------------------------------
		   1 = test passed, 0 = test failed

		   Bit 0 = Accelerometer self test
		   Bit 1 = Magnetometer self test
		   Bit 2 = Gyroscope self test
		   Bit 3 = MCU self test

		   0x0F = all good! */

		status.self_test_result = read8(reg_t.BNO055_SELFTEST_RESULT_ADDR);

		/* System Error (see section 4.3.59)
		   ---------------------------------
		   0 = No error
		   1 = Peripheral initialization error
		   2 = System initialization error
		   3 = Self test result failed
		   4 = Register map value out of range
		   5 = Register map address out of range
		   6 = Register map write error
		   7 = BNO low power mode not available for selected operation mode
		   8 = Accelerometer power mode not available
		   9 = Fusion algorithm configuration error
		   A = Sensor configuration error */
		status.system_error = read8(reg_t.BNO055_SYS_ERR_ADDR);
		return status;
	}

	/**
	 * Gets the chip revision numbers
	 *
	 * @return the chips revision information
	 */
	public RevInfo getRevInfo() {
		int a = 0, b = 0;
		RevInfo info = new RevInfo();

		/* Check the accelerometer revision */
		info.accel_rev = read8(reg_t.BNO055_ACCEL_REV_ID_ADDR);

		/* Check the magnetometer revision */
		info.mag_rev   = read8(reg_t.BNO055_MAG_REV_ID_ADDR);

		/* Check the gyroscope revision */
		info.gyro_rev  = read8(reg_t.BNO055_GYRO_REV_ID_ADDR);

		/* Check the SW revision */
		info.bl_rev    = read8(reg_t.BNO055_BL_REV_ID_ADDR);

		a = read8(reg_t.BNO055_SW_REV_ID_LSB_ADDR);
		b = read8(reg_t.BNO055_SW_REV_ID_MSB_ADDR);
		info.sw_rev = (short) ((b << 8) | a);

		return info;
	}

	/**
	 * Diagnostic method to determine if communications with the sensor are active.
	 *   Note this method returns true after first establishing communications
	 *   with the sensor.
	 *   Communications are not actively monitored once sensor initialization
	 *     has started.
	 * @return true if the sensor is found on the I2C bus
	 */
	public boolean isSensorPresent() {
		return sensorPresent;
	}

	/**
	 * After power is applied, the sensor needs to be configured for use.
	 *   During this initialization period the sensor will not return position
	 *   vector data. Once initialization is complete, data can be read,
	 *   although the sensor may not have completed calibration.
	 *   See isCalibrated. 
	 * @return true when the sensor is initialized.
	 */
	public boolean isInitialized() {
		return initialized;
	}
	
	/**
	 * Gets current calibration state.
	 * @return each value will be set to 0 if not calibrated, 3 if fully
	 *   calibrated.
	 */
	public CalData getCalibration() {
		CalData data = new CalData();
		int rawCalData = read8(reg_t.BNO055_CALIB_STAT_ADDR);

		data.sys = (byte) ((rawCalData >> 6) & 0x03);
		data.gyro = (byte) ((rawCalData >> 4) & 0x03);
		data.accel = (byte) ((rawCalData >> 2) & 0x03);
		data.mag = (byte) (rawCalData & 0x03);

		return data;
	}

	/**
	 * Returns true if all required sensors (accelerometer, magnetometer,
	 *   gyroscope) have completed their respective calibration sequence.
	 *   Only sensors required by the current operating mode are checked.
	 *   See Section 3.3.
	 * @return true if calibration is complete for all sensors required for the
	 *   mode the sensor is currently operating in. 
	 */
	public boolean isCalibrated() {
		boolean retVal = true;
		
		//Per Table 3-3
		boolean[][] sensorModeMap = new boolean[][]{
			//{accel, mag, gyro}
			{false, false, false}, // OPERATION_MODE_CONFIG
			{ true, false, false}, // OPERATION_MODE_ACCONLY
			{false,  true, false}, // OPERATION_MODE_MAGONLY
			{false, false,  true}, // OPERATION_MODE_GYRONLY
			{ true,  true, false}, // OPERATION_MODE_ACCMAG
			{ true, false,  true}, // OPERATION_MODE_ACCGYRO
			{false,  true,  true}, // OPERATION_MODE_MAGGYRO
			{ true,  true,  true}, // OPERATION_MODE_AMG
			{ true, false,  true}, // OPERATION_MODE_IMUPLUS
			{ true,  true, false}, // OPERATION_MODE_COMPASS
			{ true,  true, false}, // OPERATION_MODE_M4G
			{ true,  true,  true}, // OPERATION_MODE_NDOF_FMC_OFF
			{ true,  true,  true}  // OPERATION_MODE_NDOF
		};

		CalData data = getCalibration();
		
		if(sensorModeMap[_mode][0]) //Accelerometer used
			retVal = retVal && (data.accel >= 3);
		if(sensorModeMap[_mode][1]) //Magnetometer used
			retVal = retVal && (data.mag >= 3);
		if(sensorModeMap[_mode][2]) //Gyroscope used
			retVal = retVal && (data.gyro >= 3);
		
		return retVal;
	}
	
	/**
	 * Get the sensors internal temperature.
	 * @return temperature in degrees celsius.
	 */
	public int getTemp() {
		return (read8(reg_t.BNO055_TEMP_ADDR));
	}

	/**
	 * Gets a vector representing the sensors position (heading, roll, pitch).
	 * heading:    0 to 360 degrees
	 * roll:     -90 to +90 degrees
	 * pitch:   -180 to +180 degrees
	 *
	 * For continuous rotation heading (doesn't roll over between 360/0) see
	 *   the getHeading() method.
	 *
	 * Maximum data output rates for Fusion modes - See 3.6.3
	 * 
	 * Operating Mode		Data Output Rate
	 *   IMU                  100 Hz
	 *   COMPASS               20 Hz
	 *   M4G                   50 Hz
	 *   NDOF_FMC_OFF         100 Hz
	 *   NDOF                 100 Hz
	 *
	 * @return a vector [heading, roll, pitch]
	 */
	public double[] getVector() {
		return xyz;
	}
	
	/**
	 * The heading of the sensor (x axis) in continuous format. Eg rotating the
	 *   sensor clockwise two full rotations will return a value of 720 degrees.
	 * The getVector method will return heading in a constrained 0 - 360 deg
	 *   format if required. Does not apply an offset to the result.
	 * @return heading in degrees
	 */
	public double getHeading() {
		return xyz[0] + turns * 360;
	}
	
	/**
	 * Writes an 8 bit value over I2C
	 * @param reg the register to write the data to
	 * @param value a byte of data to write
	 * @return whatever I2CJNI.i2CWrite returns. It's not documented in the wpilib javadocs!
	 */
	private boolean write8(reg_t reg, byte value) {
		boolean retVal = false;

		retVal = imu.write(reg.getVal(), value);

		return retVal;
	}

	/**
	 * Reads an 8 bit value over I2C
	 * @param reg the register to read from.
	 * @return
	 */
	private byte read8(reg_t reg) {
		byte[] vals = new byte[1];

		readLen(reg, vals);
		return vals[0];
	}

	/**
	 * Reads the specified number of bytes over I2C
	 *
	 * @param reg the address to read from
	 * @param buffer to store the read data into
	 * @return true on success
	 */
	private boolean readLen(reg_t reg, byte[] buffer) {
		return readLen(reg.getVal(), buffer);
	}

	/**
	 * Reads the specified number of bytes over I2C
	 *
	 * @param reg the address to read from
	 * @param buffer the size of the data to read
	 * @return true on success
	 */
	private boolean readLen(int reg, byte[] buffer) {
		boolean retVal = true;

		if (buffer == null || buffer.length < 1) {
			return false;
		}

		retVal = !imu.read(reg, buffer.length, buffer);

		return retVal;
	}
	
	private class BNO055UpdateTask extends TimerTask {
		private BNO055 imu;

		private BNO055UpdateTask(BNO055 imu) {
			if (imu == null) {
				throw new NullPointerException("BNO055 pointer null");
			}
			this.imu = imu;
		}

		/**
		 * Called periodically in its own thread
		 */
		public void run() {
			imu.update();
		}
	}

	/**
	 * prints out system status during initialization
	 */
	private void printSystemStatus() {
		SystemStatus sysStat = getSystemStatus();
		println(
		  String.format(
			"system: status: %d, test result %d, error %d",
			sysStat.system_status,
			sysStat.self_test_result,
			sysStat.system_error
		  )
		);
	}

	public static class BNO055OffsetData {
		public int accelOffsetX = 0;
		public int accelOffsetY = 0;
		public int accelOffsetZ = 0;
		public int accelRadius = 0;

		public int gyroOffsetX = 0;
		public int gyroOffsetY = 0;
		public int gyroOffsetZ = 0;

		public int magOffsetX = 0;
		public int magOffsetY = 0;
		public int magOffsetZ = 0;
		public int magRadius = 0;

		public BNO055OffsetData() {}

		public BNO055OffsetData(int ax, int ay, int az, int ar, int gx, int gy, int gz, int mx, int my, int mz, int mr) {
			accelOffsetX = ax;
			accelOffsetY = ay;
			accelOffsetZ = az;
			accelRadius = ar;
			gyroOffsetX = gx;
			gyroOffsetY = gy;
			gyroOffsetZ = gz;
			magOffsetX = mx;
			magOffsetY = my;
			magOffsetZ = mz;
			magRadius = mr;
		}
	}

	private void write16(reg_t registerLSB, reg_t registerMSB, int value) {
		// Write the least significant bits
		write8(registerLSB, (byte) (value & 0x00FF));
		// Write the most significant bits
		write8(registerMSB, (byte) ((value >> 8) & 0x00FF));
	}
	private int read16(reg_t registerLSB, reg_t registerMSB) {
		int value = 0;
		// Read the least significant bits
		int lsb = read8(registerLSB);
		// Read the most significant bits
		int msb = read8(registerMSB);
		value = (msb << 8) | lsb;
		return value;
	}

	public void writeOffsets(BNO055OffsetData offsetData) {
		println("setting calibration offsets.");
		// Set device to config mode
		setMode(opmode_t.OPERATION_MODE_CONFIG);
		// write calib registers
		write16(reg_t.ACCEL_OFFSET_X_LSB_ADDR, reg_t.ACCEL_OFFSET_X_MSB_ADDR, offsetData.accelOffsetX);
		write16(reg_t.ACCEL_OFFSET_Y_LSB_ADDR, reg_t.ACCEL_OFFSET_Y_MSB_ADDR, offsetData.accelOffsetY);
		write16(reg_t.ACCEL_OFFSET_Z_LSB_ADDR, reg_t.ACCEL_OFFSET_Z_MSB_ADDR, offsetData.accelOffsetZ);
		write16(reg_t.ACCEL_RADIUS_LSB_ADDR, reg_t.ACCEL_RADIUS_MSB_ADDR, offsetData.accelRadius);

		write16(reg_t.GYRO_OFFSET_X_LSB_ADDR, reg_t.GYRO_OFFSET_X_MSB_ADDR, offsetData.gyroOffsetX);
		write16(reg_t.GYRO_OFFSET_Y_LSB_ADDR, reg_t.GYRO_OFFSET_Y_MSB_ADDR, offsetData.gyroOffsetY);
		write16(reg_t.GYRO_OFFSET_Z_LSB_ADDR, reg_t.GYRO_OFFSET_Z_MSB_ADDR, offsetData.gyroOffsetZ);

		write16(reg_t.MAG_OFFSET_X_LSB_ADDR, reg_t.MAG_OFFSET_X_MSB_ADDR, offsetData.magOffsetX);
		write16(reg_t.MAG_OFFSET_Y_LSB_ADDR, reg_t.MAG_OFFSET_Y_MSB_ADDR, offsetData.magOffsetY);
		write16(reg_t.MAG_OFFSET_Z_LSB_ADDR, reg_t.MAG_OFFSET_Z_MSB_ADDR, offsetData.magOffsetZ);
		write16(reg_t.MAG_RADIUS_LSB_ADDR, reg_t.MAG_RADIUS_MSB_ADDR, offsetData.magRadius);
		
		println("calibration offsets written.");
	}

	public BNO055OffsetData readOffsets() {
		// Set device to config mode
		setMode(opmode_t.OPERATION_MODE_CONFIG);
		
		BNO055OffsetData offsets = new BNO055OffsetData();
		offsets.accelOffsetX = read16(reg_t.ACCEL_OFFSET_X_LSB_ADDR, reg_t.ACCEL_OFFSET_X_MSB_ADDR);
		offsets.accelOffsetY = read16(reg_t.ACCEL_OFFSET_Y_LSB_ADDR, reg_t.ACCEL_OFFSET_Y_MSB_ADDR);
		offsets.accelOffsetZ = read16(reg_t.ACCEL_OFFSET_Z_LSB_ADDR, reg_t.ACCEL_OFFSET_Z_MSB_ADDR);
		offsets.accelRadius = read16(reg_t.ACCEL_RADIUS_LSB_ADDR, reg_t.ACCEL_RADIUS_MSB_ADDR);

		offsets.gyroOffsetX = read16(reg_t.GYRO_OFFSET_X_LSB_ADDR, reg_t.GYRO_OFFSET_X_MSB_ADDR);
		offsets.gyroOffsetY = read16(reg_t.GYRO_OFFSET_Y_LSB_ADDR, reg_t.GYRO_OFFSET_Y_MSB_ADDR);
		offsets.gyroOffsetZ = read16(reg_t.GYRO_OFFSET_Z_LSB_ADDR, reg_t.GYRO_OFFSET_Z_MSB_ADDR);

		offsets.magOffsetX = read16(reg_t.MAG_OFFSET_X_LSB_ADDR, reg_t.MAG_OFFSET_X_MSB_ADDR);
		offsets.magOffsetY = read16(reg_t.MAG_OFFSET_Y_LSB_ADDR, reg_t.MAG_OFFSET_Y_MSB_ADDR);
		offsets.magOffsetZ = read16(reg_t.MAG_OFFSET_Z_LSB_ADDR, reg_t.MAG_OFFSET_Z_MSB_ADDR);
		offsets.magRadius = read16(reg_t.MAG_RADIUS_LSB_ADDR, reg_t.MAG_RADIUS_MSB_ADDR);

		println("offsets:");
		println("accelx: " + offsets.accelOffsetX);
		println("accely: " + offsets.accelOffsetY);
		println("accelz: " + offsets.accelOffsetZ);
		println("accelradius: " + offsets.accelRadius);
	
		println("gyrox: " + offsets.gyroOffsetX);
		println("gyroy: " + offsets.gyroOffsetY);
		println("gyroz: " + offsets.gyroOffsetZ);
	
		println("magx: " + offsets.magOffsetX);
		println("magy: " + offsets.magOffsetY);
		println("magz: " + offsets.magOffsetZ);
		println("magradius: " + offsets.magRadius);

		return offsets;
	}

	// These methods made to simulate pigeon-esque behavior
	/**
	 * Get heading of device (yaw while in heading mode)
	 */
	public Rotation2d getRotation2d() {
		// get yaw
		return Rotation2d.fromDegrees(getHeading() - headingOffset);
	}

	/**
	 * Sets the heading offset to current heading.
	 */
	public void reset() {
		headingOffset = getHeading();
	}

	private void println(String message) {
		System.out.println(name + " " + message);
	}

	public void log() {
		SmartDashboard.putBoolean(name + " " + "initialized", isInitialized());
		if (isInitialized()) {
			// Reading this stuff while initializing can be hazardous to the startup process
			CalData cal = getCalibration();
			SmartDashboard.putNumber(name + " " + "gyro calibration", cal.gyro);
			SmartDashboard.putNumber(name + " " + "accel calibration", cal.accel);
			SmartDashboard.putNumber(name + " " + "mag calibration", cal.mag);
			SmartDashboard.putNumber(name + " " + "sys calibration", cal.sys);
			SystemStatus stat = getSystemStatus();
			SmartDashboard.putNumber(name + " " + "status", stat.system_status);
			SmartDashboard.putNumber(name + " " + "self test", stat.self_test_result);
			SmartDashboard.putNumber(name + " " + "error", stat.system_error);
			double[] vec = getVector();
			SmartDashboard.putNumber(name + " " + "X", vec[0]);
			SmartDashboard.putNumber(name + " " + "Y", vec[1]);
			SmartDashboard.putNumber(name + " " + "Z", vec[2]);
		}
	} 
}