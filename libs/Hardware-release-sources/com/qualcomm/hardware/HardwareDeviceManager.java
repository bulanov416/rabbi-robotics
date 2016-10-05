/*
 * Copyright (c) 2014, 2015 Qualcomm Technologies Inc
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * (subject to the limitations in the disclaimer below) provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Qualcomm Technologies Inc nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE. THIS
 * SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.qualcomm.hardware;

import android.content.Context;

import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.ams.AMSColorSensorImpl;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtAccelerationSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtColorSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtCompassSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtDcMotorController;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtIrSeekerSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtLightSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtServoController;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtTouchSensor;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtTouchSensorMultiplexer;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtUltrasonicSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsDigitalTouchSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDevice;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbServoController;
import com.qualcomm.modernrobotics.ModernRoboticsUsbUtil;
import com.qualcomm.modernrobotics.RobotUsbManagerEmulator;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.AnalogOutputController;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceImpl;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputImpl;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.PWMOutputImplEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.configuration.UserSensorType;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.hardware.usb.RobotUsbManager;
import com.qualcomm.robotcore.hardware.usb.ftdi.RobotUsbDeviceFtdi;
import com.qualcomm.robotcore.hardware.usb.ftdi.RobotUsbManagerFtdi;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.ThreadPool;

import java.lang.reflect.InvocationTargetException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Scan for, and create instances of, Modern Robotics USB devices
 */
public class HardwareDeviceManager implements DeviceManager {

  //------------------------------------------------------------------------------------------------
  // Types
  //------------------------------------------------------------------------------------------------

  private enum Mode {
    DEFAULT,
    ENABLE_DEVICE_EMULATION
  }

  //------------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------------

  public static final String TAG = "HardwareDeviceManager";
  public static final String TAG_USB_SCAN = "USBScan";

  private static Mode mode = Mode.DEFAULT;

  public  final static Object     scanDevicesLock = new Object();

  private       RobotUsbManager   usbManager;
  private final EventLoopManager  manager;
  private final Context           context;

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  /**
   * ModernRoboticsUsbDeviceManager constructor
   * @param context Context of current Android app
   * @param manager event loop manager
   * @throws RobotCoreException if unable to open FTDI D2XX manager
   */
   public HardwareDeviceManager(Context context, EventLoopManager manager) throws RobotCoreException {
    this.context = context;
    this.manager = manager;

    switch (mode) {
      case ENABLE_DEVICE_EMULATION:
        usbManager = new RobotUsbManagerEmulator();
        break;
      default:
        usbManager = new RobotUsbManagerFtdi(context);
    }
  }

  //------------------------------------------------------------------------------------------------
  // Scanning
  //------------------------------------------------------------------------------------------------

/* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#scanForUsbDevices()
   *
   * Returns a map from serial number to DeviceType
   */
  @Override
  public Map<SerialNumber, DeviceType> scanForUsbDevices() throws RobotCoreException {
    synchronized (scanDevicesLock) {
      long start = System.nanoTime();
      final Map<SerialNumber, DeviceType> deviceMap = new ConcurrentHashMap<SerialNumber, DeviceType>();
      int devCount = usbManager.scanForDevices();
      usbManager.freezeScanForDevices();  // Prevent ModernRoboticsUsbUtil.openUsbDevice from issuing concurrent usbManager.scanForDevices() calls
      try {
        if (devCount > 0) {
          // Open all the USB devices attached to the robot controller. We do this in parallel so as to minimize latency to the user.
          ExecutorService executorService = ThreadPool.newFixedThreadPool(devCount);
          final ConcurrentHashMap<SerialNumber, RobotUsbDevice> newlyFoundDevices = new ConcurrentHashMap<SerialNumber, RobotUsbDevice>();
          try {
            for (int id = 0; id < devCount; id++) {
              final SerialNumber serialNumber = usbManager.getDeviceSerialNumberByIndex(id);
              executorService.execute(new Runnable() {
                @Override public void run() {
                    try {
                      RobotLog.vv(TAG_USB_SCAN, "opening %s...", serialNumber);
                      RobotUsbDevice device = ModernRoboticsUsbUtil.openUsbDevice(usbManager, serialNumber);
                      RobotLog.vv(TAG_USB_SCAN, "... done opening %s", serialNumber);
                      newlyFoundDevices.put(serialNumber, device);
                    } catch (RobotCoreException e) {
                      RobotLog.vv(TAG_USB_SCAN, "failed opening %s", serialNumber.toString());
                    }
                  }
                }
              );
            }
            // Wait for all those opens to finish
            executorService.shutdown();
            ThreadPool.awaitTerminationOrExitApplication(executorService, 30, TimeUnit.SECONDS, "USB Scanning Service", "internal error");

            // Having opened everything, determine the type of each
            for (Map.Entry<SerialNumber,RobotUsbDevice> pair : newlyFoundDevices.entrySet()) {
              determineModernRoboticsDeviceType(pair.getValue(), pair.getKey(), deviceMap);
            }

            // Also consider devices that are already open
            for (RobotUsbDevice existingDevice : RobotUsbDeviceFtdi.getExtantDevices()) {
              SerialNumber serialNumber = existingDevice.getSerialNumber();
              if (!newlyFoundDevices.containsKey(serialNumber)) {
                DeviceType deviceType = existingDevice.getDeviceType();
                if (deviceType != DeviceType.FTDI_USB_UNKNOWN_DEVICE) {
                  RobotLog.vv(TAG_USB_SCAN, "added extant device %s type=%s", serialNumber.toString(), deviceType.toString());
                  deviceMap.put(serialNumber, deviceType);
                }
              }
            }

          } finally {
            // On the way out, be sure to close all.
            for (Map.Entry<SerialNumber,RobotUsbDevice> pair : newlyFoundDevices.entrySet()) {
              RobotLog.vv(TAG_USB_SCAN, "closing %s", pair.getKey());
              pair.getValue().close();
            }
          }
        }
      } finally {
        usbManager.thawScanForDevices();
      }
      long end = System.nanoTime();
      RobotLog.vv(TAG_USB_SCAN, "scanForUsbDevices() took %dms count=%d", (int)((end-start) / ElapsedTime.MILLIS_IN_NANO), deviceMap.size());
      return deviceMap;
    }
  }

  void determineModernRoboticsDeviceType(RobotUsbDevice dev, SerialNumber serialNumber, Map<SerialNumber, DeviceType> deviceMap) {
  // Open the indicated MR device by serial number in order to determine its device type
    DeviceType deviceType = DeviceType.UNKNOWN_DEVICE;
    RobotUsbDevice.USBIdentifiers ids = dev.getUsbIdentifiers();
    if (ids.isModernRoboticsDevice()) {
      try {
        RobotLog.vv(TAG_USB_SCAN, "getting MR device device header %s ...", serialNumber);
        deviceType = getModernRoboticsDeviceType(dev);
        RobotLog.vv(TAG_USB_SCAN, "... done getting MR device device header %s type=%s", serialNumber, deviceType);
      } catch (RobotCoreException ignored) {
        RobotLog.vv(TAG_USB_SCAN, "exception retrieving MR device device header %s", serialNumber);
        return;
      }
    } else {
      // we can't figure this guy out; ignore
      return;
    }
    //
    deviceMap.put(serialNumber, deviceType);
  }

  DeviceType getModernRoboticsDeviceType(RobotUsbDevice dev) throws RobotCoreException {
    byte[] modernRoboticsDeviceHeader = getModernRoboticsDeviceHeader(dev);
    return getModernRoboticsDeviceType(dev, modernRoboticsDeviceHeader);
  }

  DeviceType getModernRoboticsDeviceType(RobotUsbDevice dev, byte[] modernRoboticsDeviceHeader) throws RobotCoreException {
    DeviceType deviceType = ModernRoboticsUsbUtil.getDeviceType(modernRoboticsDeviceHeader);
    // Record the device type so we can retreive it later w/o needing to open USB again
    dev.setDeviceType(deviceType);
    return deviceType;
  }

  byte[] getModernRoboticsDeviceHeader(RobotUsbDevice dev) throws RobotCoreException {
    return ModernRoboticsUsbUtil.getUsbDeviceHeader(dev);
  }

  //------------------------------------------------------------------------------------------------
  // Creation
  //------------------------------------------------------------------------------------------------

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createUsbDcMotorController(com.qualcomm.robotcore.util.SerialNumber)
   */
  @Override
  public DcMotorController createUsbDcMotorController(final SerialNumber serialNumber)
      throws RobotCoreException, InterruptedException {
    HardwareFactory.noteSerialNumberType(context, serialNumber, context.getString(R.string.moduleDisplayNameMotorController));
    RobotLog.v("Creating %s", HardwareFactory.getDeviceDisplayName(context, serialNumber));

    ModernRoboticsUsbDevice.OpenRobotUsbDevice openRobotUsbDevice = new ModernRoboticsUsbDevice.OpenRobotUsbDevice() {
      @Override public RobotUsbDevice open() throws RobotCoreException, InterruptedException {
          RobotUsbDevice dev = null;
          try {
            dev = ModernRoboticsUsbUtil.openUsbDevice(usbManager, serialNumber);
            byte[] deviceHeader = getModernRoboticsDeviceHeader(dev);
            DeviceType type = getModernRoboticsDeviceType(dev, deviceHeader);

            if (type != DeviceType.MODERN_ROBOTICS_USB_DC_MOTOR_CONTROLLER) {
              closeAndThrowOnFailedDeviceTypeCheck(dev, serialNumber);
            }
            dev.setFirmwareVersion(getModernRoboticsFirmwareVersion(deviceHeader));
          } catch (RobotCoreException e) {
            if (dev != null) dev.close(); // avoid leakage of open FT_Devices
            throw e;
          } catch (RuntimeException e) {
            if (dev != null) dev.close(); // avoid leakage of open FT_Devices
            throw e;
          }
          return dev;
        }
    };

    ModernRoboticsUsbDcMotorController controller = new ModernRoboticsUsbDcMotorController(context, serialNumber, openRobotUsbDevice, manager);
    controller.armOrPretend();
    controller.initializeHardware();
    return controller;
  }

  @Override
  public DcMotor createDcMotor(DcMotorController controller, int portNumber) {
    return new DcMotorImpl(controller, portNumber, DcMotor.Direction.FORWARD);
  }
  @Override
  public DcMotor createDcMotorEx(DcMotorController controller, int portNumber) {
    return new DcMotorImplEx(controller, portNumber, DcMotor.Direction.FORWARD);
  }


  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createUsbServoController(com.qualcomm.robotcore.util.SerialNumber)
   */
  @Override
  public ServoController createUsbServoController(final SerialNumber serialNumber)
      throws RobotCoreException, InterruptedException {
    HardwareFactory.noteSerialNumberType(context, serialNumber, context.getString(R.string.moduleDisplayNameServoController));
    RobotLog.v("Creating %s", HardwareFactory.getDeviceDisplayName(context, serialNumber));

    ModernRoboticsUsbDevice.OpenRobotUsbDevice openRobotUsbDevice = new ModernRoboticsUsbDevice.OpenRobotUsbDevice() {
      @Override public RobotUsbDevice open() throws RobotCoreException, InterruptedException {
        RobotUsbDevice dev = null;
        try {
          dev = ModernRoboticsUsbUtil.openUsbDevice(usbManager, serialNumber);
          byte[] deviceHeader = getModernRoboticsDeviceHeader(dev);
          DeviceType type = getModernRoboticsDeviceType(dev, deviceHeader);

          if (type != DeviceType.MODERN_ROBOTICS_USB_SERVO_CONTROLLER) {
            closeAndThrowOnFailedDeviceTypeCheck(dev, serialNumber);
          }
          dev.setFirmwareVersion(getModernRoboticsFirmwareVersion(deviceHeader));
        } catch (RobotCoreException e) {
          if (dev != null) dev.close(); // avoid leakage of open FT_Devices
          throw e;
        } catch (RuntimeException e) {
          if (dev != null) dev.close(); // avoid leakage of open FT_Devices
          throw e;
        }
        return dev;
      }
    };

    ModernRoboticsUsbServoController controller = new ModernRoboticsUsbServoController(context, serialNumber, openRobotUsbDevice, manager);
    controller.armOrPretend();
    controller.initializeHardware();
    return controller;
  }

  @Override
  public Servo createServo(ServoController controller, int portNumber) {
    return new ServoImpl(controller, portNumber, Servo.Direction.FORWARD);
  }

  @Override
  public CRServo createCRServo(ServoController controller, int portNumber) {
    return new CRServoImpl(controller, portNumber, DcMotor.Direction.FORWARD);
  }

  @Override
  public Servo createServoEx(ServoController controller, int portNumber) {
    return new ServoImplEx(controller, portNumber, Servo.Direction.FORWARD);
  }

  @Override
  public DeviceInterfaceModule createDeviceInterfaceModule(final SerialNumber serialNumber)
    throws RobotCoreException, InterruptedException {
    HardwareFactory.noteSerialNumberType(context, serialNumber, context.getString(R.string.moduleDisplayNameCDIM));
    RobotLog.v("Creating %s", HardwareFactory.getDeviceDisplayName(context, serialNumber));

    ModernRoboticsUsbDevice.OpenRobotUsbDevice openRobotUsbDevice = new ModernRoboticsUsbDevice.OpenRobotUsbDevice() {
      @Override public RobotUsbDevice open() throws RobotCoreException, InterruptedException {
        RobotUsbDevice dev = null;
        try {
          dev = ModernRoboticsUsbUtil.openUsbDevice(usbManager, serialNumber);
          byte[] deviceHeader = getModernRoboticsDeviceHeader(dev);
          DeviceType type = getModernRoboticsDeviceType(dev, deviceHeader);

          if (type != DeviceType.MODERN_ROBOTICS_USB_DEVICE_INTERFACE_MODULE) {
            closeAndThrowOnFailedDeviceTypeCheck(dev, serialNumber);
          }
          dev.setFirmwareVersion(getModernRoboticsFirmwareVersion(deviceHeader));
        } catch (RobotCoreException e) {
          if (dev != null) dev.close(); // avoid leakage of open FT_Devices
          throw e;
        } catch (RuntimeException e) {
          if (dev != null) dev.close(); // avoid leakage of open FT_Devices
          throw e;
        }
        return dev;
      }
    };
    ModernRoboticsUsbDeviceInterfaceModule deviceInterfaceModule = new ModernRoboticsUsbDeviceInterfaceModule(context, serialNumber, openRobotUsbDevice, manager);
    deviceInterfaceModule.armOrPretend();
    deviceInterfaceModule.initializeHardware();
    return deviceInterfaceModule;
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createUsbLegacyModule(com.qualcomm.robotcore.util.SerialNumber)
   */
  @Override
  public LegacyModule createUsbLegacyModule(final SerialNumber serialNumber)
      throws RobotCoreException, InterruptedException {
    HardwareFactory.noteSerialNumberType(context, serialNumber, context.getString(R.string.moduleDisplayNameLegacyModule));
    RobotLog.v("Creating %s", HardwareFactory.getDeviceDisplayName(context, serialNumber));

    ModernRoboticsUsbDevice.OpenRobotUsbDevice openRobotUsbDevice = new ModernRoboticsUsbDevice.OpenRobotUsbDevice() {
      @Override public RobotUsbDevice open() throws RobotCoreException, InterruptedException {
        RobotUsbDevice dev = null;
        try {
          dev = ModernRoboticsUsbUtil.openUsbDevice(usbManager, serialNumber);
          byte[] deviceHeader = getModernRoboticsDeviceHeader(dev);
          DeviceType type = getModernRoboticsDeviceType(dev, deviceHeader);

          if (type != DeviceType.MODERN_ROBOTICS_USB_LEGACY_MODULE) {
            closeAndThrowOnFailedDeviceTypeCheck(dev, serialNumber);
          }
          dev.setFirmwareVersion(getModernRoboticsFirmwareVersion(deviceHeader));
        } catch (RobotCoreException e) {
          if (dev != null) dev.close(); // avoid leakage of open FT_Devices
          throw e;
        } catch (RuntimeException e) {
          if (dev != null) dev.close(); // avoid leakage of open FT_Devices
          throw e;
        }
        return dev;
      }
    };
    ModernRoboticsUsbLegacyModule legacyModule = new ModernRoboticsUsbLegacyModule(context, serialNumber, openRobotUsbDevice, manager);
    legacyModule.armOrPretend();
    legacyModule.initializeHardware();
    return legacyModule;
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createNxtDcMotorController(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public DcMotorController createNxtDcMotorController(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT DC Motor Controller - Port: " + physicalPort);
    return new HiTechnicNxtDcMotorController(context, legacyModule, physicalPort);
  }

/* (non-Javadoc)
 * @see com.qualcomm.hardware.DeviceManager#createNxtServoController(com.qualcomm.robotcore.hardware.LegacyModule, int)
 */
  @Override
  public ServoController createNxtServoController(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Servo Controller - Port: " + physicalPort);
    return new HiTechnicNxtServoController(context, legacyModule, physicalPort);
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createNxtCompassSensor(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public CompassSensor createNxtCompassSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Compass Sensor - Port: " + physicalPort);
    return new HiTechnicNxtCompassSensor(promote(legacyModule), physicalPort);
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createDigitalTouchSensor(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public TouchSensor createDigitalTouchSensor(DeviceInterfaceModule deviceInterfaceModule, int physicalPort) {
    RobotLog.v("Creating Modern Robotics Digital Touch Sensor - Port: " + physicalPort);
    return new ModernRoboticsDigitalTouchSensor(promote(deviceInterfaceModule), physicalPort);
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createNxtAccelerationSensor(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public AccelerationSensor createNxtAccelerationSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Acceleration Sensor - Port: " + physicalPort);
    return new HiTechnicNxtAccelerationSensor(promote(legacyModule), physicalPort);
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createNxtLightSensor(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public LightSensor createNxtLightSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Light Sensor - Port: " + physicalPort);
    return new HiTechnicNxtLightSensor(promote(legacyModule), physicalPort);
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createNxtLightSensor(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public GyroSensor createNxtGyroSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Gyro Sensor - Port: " + physicalPort);
    return new HiTechnicNxtGyroSensor(promote(legacyModule), physicalPort);
  }

  /* (non-Javadoc)
   * @see com.qualcomm.hardware.DeviceManager#createNxtIrSeekerSensor(com.qualcomm.robotcore.hardware.LegacyModule, int)
   */
  @Override
  public IrSeekerSensor createNxtIrSeekerSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT IR Seeker Sensor - Port: " + physicalPort);
    return new HiTechnicNxtIrSeekerSensor(promote(legacyModule), physicalPort);
  }

  @Override
  public IrSeekerSensor createI2cIrSeekerSensorV3(I2cController i2cController, int physicalPort) {
    RobotLog.v("Creating Modern Robotics I2C IR Seeker Sensor V3 - Port: " + physicalPort);
    return new ModernRoboticsI2cIrSeekerSensorV3(i2cController, physicalPort);
  }

  @Override
  public UltrasonicSensor createNxtUltrasonicSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Ultrasonic Sensor - Port: " + physicalPort);
    return new HiTechnicNxtUltrasonicSensor(promote(legacyModule), physicalPort);
  }

  @Override
  public OpticalDistanceSensor createAnalogOpticalDistanceSensor(AnalogInputController analogInputController, int physicalPort) {
    RobotLog.v("Creating Modern Robotics Analog Optical Distance Sensor - Port: " + physicalPort);
    return new ModernRoboticsAnalogOpticalDistanceSensor(analogInputController, physicalPort);
  }

  @Override
  public TouchSensor createNxtTouchSensor(LegacyModule legacyModule, int physicalPort) {
    RobotLog.v("Creating HiTechnic NXT Touch Sensor - Port: " + physicalPort);
    return new HiTechnicNxtTouchSensor(promote(legacyModule), physicalPort);
  }

  @Override
  public TouchSensorMultiplexer createNxtTouchSensorMultiplexer(LegacyModule legacyModule, int port) {
    RobotLog.v("Creating HiTechnic NXT Touch Sensor Multiplexer - Port: " + port);
    return new HiTechnicNxtTouchSensorMultiplexer(promote(legacyModule), port);
  }

  @Override
  public AnalogInput createAnalogInputDevice(AnalogInputController controller, int channel) {
    RobotLog.v("Creating Analog Input Device - Port: " + channel);
    return new AnalogInput(controller, channel);
  }

  @Override
  public AnalogOutput createAnalogOutputDevice(AnalogOutputController controller, int channel) {
    RobotLog.v("Creating Analog Output Device - Port: " + channel);
    return new AnalogOutput(controller, channel);
  }

  @Override
  public DigitalChannel createDigitalChannelDevice(DigitalChannelController controller, int channel) {
    RobotLog.v("Creating Digital Channel Device - Port: " + channel);
    return new DigitalChannel(controller, channel);
  }

  @Override
  public PWMOutput createPwmOutputDevice(PWMOutputController controller, int channel) {
    RobotLog.v("Creating PWM Output Device - Port: " + channel);
    return new PWMOutputImpl(controller, channel);
  }

  @Override
  public PWMOutput createPwmOutputDeviceEx(PWMOutputController controller, int channel) {
    RobotLog.v("Creating PWM Output Device - Port: " + channel);
    return new PWMOutputImplEx(controller, channel);
  }

  @Override
  public I2cDevice createI2cDevice(I2cController controller, int channel) {
    RobotLog.v("Creating I2C Device - Port: " + channel);
    return new I2cDeviceImpl(controller, channel);
  }

  @Override
  public HardwareDevice createUserI2cDevice(I2cController controller, int channel, UserSensorType type) {
    RobotLog.v("Creating user sensor %s - Port: %d", type.getName(), channel);
    try {
      return type.createInstance(controller, channel);
    } catch (InvocationTargetException e) {
      RobotLog.v("Creating user sensor %s failed: ", type.getName());
      Exception eToLog = e.getTargetException()!=null && (e.getTargetException() instanceof Exception) ? ((Exception)e.getTargetException()) : e;
      RobotLog.logStacktrace(eToLog);
      return null;
    }
  }

  @Override
  public ColorSensor createAdafruitI2cColorSensor(I2cController controller, int channel) {
    RobotLog.v("Creating Adafruit I2C Color Sensor - Port: " + channel);
    return new AdafruitI2cColorSensor(controller, channel);
  }

  @Override
  public ColorSensor createNxtColorSensor(LegacyModule controller, int channel) {
    RobotLog.v("Creating HiTechnic NXT Color Sensor - Port: " + channel);
    return new HiTechnicNxtColorSensor(controller, channel);
  }

  @Override
  public ColorSensor createModernRoboticsI2cColorSensor(I2cController controller, int channel) {
    RobotLog.v("Creating Modern Robotics I2C Color Sensor - Port: " + channel);
    return new ModernRoboticsI2cColorSensor(controller, channel);
  }

  @Override
  public GyroSensor createModernRoboticsI2cGyroSensor(I2cController controller, int channel) {
    RobotLog.v("Creating Modern Robotics I2C Gyro Sensor - Port: " + channel);
    return new ModernRoboticsI2cGyro(controller, channel);
  }

  @Override
  public LED createLED(DigitalChannelController controller, int channel) {
    RobotLog.v("Creating LED - Port: " + channel);
    return new LED(controller, channel);
  }

  //------------------------------------------------------------------------------------------------
  // Utility
  //------------------------------------------------------------------------------------------------

  private RobotUsbDevice.FirmwareVersion getModernRoboticsFirmwareVersion(byte[] modernRoboticsDeviceHeader) {
    RobotUsbDevice.FirmwareVersion result = new RobotUsbDevice.FirmwareVersion();
    result.majorVersion = (modernRoboticsDeviceHeader[0] >> 4) & 0x0F;
    result.minorVersion = (modernRoboticsDeviceHeader[0] >> 0) & 0x0F;
    return result;
  }

  /**
   * Enable device emulation
   */
  public static void enableDeviceEmulation() {
    mode = Mode.ENABLE_DEVICE_EMULATION;
  }

  /**
   * Disable device emulation
   */
  public static void disableDeviceEmulation() {
    mode = Mode.DEFAULT;
  }

  private ModernRoboticsUsbLegacyModule promote(LegacyModule module) {
    if (!(module instanceof ModernRoboticsUsbLegacyModule)) {
      throw new IllegalArgumentException("Modern Robotics Device Manager needs a Modern Robotics LegacyModule");
    }

    return (ModernRoboticsUsbLegacyModule) module;
  }

  private ModernRoboticsUsbDeviceInterfaceModule promote(DeviceInterfaceModule module) {
    if (!(module instanceof ModernRoboticsUsbDeviceInterfaceModule)) {
      throw new IllegalArgumentException("Modern Robotics Device Manager needs a Modern Robotics Device Interface Module");
    }

    return (ModernRoboticsUsbDeviceInterfaceModule) module;
  }

  private void closeAndThrowOnFailedDeviceTypeCheck(RobotUsbDevice dev, SerialNumber serialNumber) throws RobotCoreException {
    String msg = String.format("%s is returning garbage data on the USB bus", HardwareFactory.getDeviceDisplayName(context, serialNumber));
    dev.close();
    logAndThrow(msg);
  }

  private void logAndThrow(String errMsg) throws RobotCoreException {
    System.err.println(errMsg);
    throw new RobotCoreException(errMsg);
  }
}
