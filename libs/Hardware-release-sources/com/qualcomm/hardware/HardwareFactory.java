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

import com.qualcomm.hardware.hitechnic.HiTechnicNxtDcMotorController;
import com.qualcomm.hardware.matrix.MatrixDcMotorController;
import com.qualcomm.hardware.matrix.MatrixMasterController;
import com.qualcomm.hardware.matrix.MatrixServoController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbLegacyModule;
import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogOutput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.TouchSensorMultiplexer;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.BuiltInConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.ControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceConfiguration;
import com.qualcomm.robotcore.hardware.configuration.DeviceInterfaceModuleConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MatrixControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.ReadXMLFileHandler;
import com.qualcomm.robotcore.hardware.configuration.ServoControllerConfiguration;
import com.qualcomm.robotcore.hardware.configuration.UserSensorType;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;

import org.xmlpull.v1.XmlPullParser;

import java.util.HashMap;
import java.util.List;

/**
 * Modern Robotics hardware factory.
 *
 * Populates the hardware map with Modern Robotics devices.
 */
@SuppressWarnings("unused")
public class HardwareFactory {

  //------------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------------

  private Context       context;
  private XmlPullParser xmlPullParser = null;

  protected static final HashMap<String,String> deviceDisplayNames = new HashMap<String, String>();

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  public HardwareFactory(Context context) {
    this.context = context;
  }

  //------------------------------------------------------------------------------------------------
  // Hardware management
  //------------------------------------------------------------------------------------------------

  /**
   * Create a hardware map
   *
   * @return HardwareMap
   */
  public HardwareMap createHardwareMap(EventLoopManager manager) throws RobotCoreException, InterruptedException  {

    // We synchronize with scanning so that there's only one thread trying to open *new* FTDI devices at a time
    synchronized (HardwareDeviceManager.scanDevicesLock) {
      HardwareMap map = new HardwareMap(context);

      if (xmlPullParser != null) {
        DeviceManager deviceMgr = new HardwareDeviceManager(context, manager);

        ReadXMLFileHandler readXmlFileHandler = new ReadXMLFileHandler();

        List<ControllerConfiguration> ctrlConfList = readXmlFileHandler.parse(xmlPullParser);

        for (ControllerConfiguration ctrlConf : ctrlConfList) {
          ConfigurationType type = ctrlConf.getType();
          if (type==BuiltInConfigurationType.MOTOR_CONTROLLER) {
            mapUsbMotorController(map, deviceMgr, ctrlConf);
          }
          else if (type==BuiltInConfigurationType.SERVO_CONTROLLER) {
            mapUsbServoController(map, deviceMgr, ctrlConf);
          }
          else if (type==BuiltInConfigurationType.LEGACY_MODULE_CONTROLLER) {
            mapUsbLegacyModule(map, deviceMgr, ctrlConf);
          }
          else if (type==BuiltInConfigurationType.DEVICE_INTERFACE_MODULE) {
            mapCoreInterfaceDeviceModule(map, deviceMgr, ctrlConf);
          }
          else {
            RobotLog.w("Unexpected controller type while parsing XML: " + type.toString());
          }
        }
      } else {
        // no XML to parse, just return empty map
      }
      return map;
    }
  }

  public void setXmlPullParser(XmlPullParser xmlPullParser) {
    this.xmlPullParser = xmlPullParser;
  }

  public XmlPullParser getXmlPullParser() {
    return xmlPullParser;
  }

  /**
   * Enable device emulation
   *
   * @throws UnsupportedOperationException if this hardware factory does not support device emulation
   */
  public static void enableDeviceEmulation() {
    HardwareDeviceManager.enableDeviceEmulation();
  }

  /**
   * Disable device emulation
   */
  public static void disableDeviceEmulation() {
    HardwareDeviceManager.disableDeviceEmulation();
  }

  private void mapUsbMotorController(HardwareMap map, DeviceManager deviceMgr, ControllerConfiguration ctrlConf) throws RobotCoreException, InterruptedException {
    if (!ctrlConf.isEnabled()) return;
    ModernRoboticsUsbDcMotorController dcMotorController = (ModernRoboticsUsbDcMotorController) deviceMgr.createUsbDcMotorController(ctrlConf.getSerialNumber());
    map.dcMotorController.put(ctrlConf.getName(), dcMotorController);
    for (DeviceConfiguration devConf : ctrlConf.getDevices()) {
      mapMotor(map, deviceMgr, devConf, dcMotorController);
    }

    VoltageSensor voltageSensor = dcMotorController;
    map.voltageSensor.put(ctrlConf.getName(), voltageSensor);
  }

  private void mapUsbServoController(HardwareMap map, DeviceManager deviceMgr, ControllerConfiguration ctrlConf) throws RobotCoreException, InterruptedException {
    if (!ctrlConf.isEnabled()) return;
    ServoController servoController = deviceMgr.createUsbServoController(ctrlConf.getSerialNumber());
    map.servoController.put(ctrlConf.getName(), servoController);
    for (DeviceConfiguration servoConf : ctrlConf.getDevices()) {
      mapServo(map, deviceMgr, servoConf, servoController);
    }
  }

  private void mapMotor(HardwareMap map, DeviceManager deviceMgr, DeviceConfiguration motorConf, DcMotorController dcMotorController) {
    if (!motorConf.isEnabled()) return;
    DcMotor dcMotor = deviceMgr.createDcMotor(dcMotorController, motorConf.getPort());
    map.dcMotor.put(motorConf.getName(), dcMotor);
  }

  private void mapServo(HardwareMap map, DeviceManager deviceMgr, DeviceConfiguration servoConf, ServoController servoController) {
    if (!servoConf.isEnabled()) return;
    if (servoConf.getType() == BuiltInConfigurationType.SERVO) {
      Servo s = deviceMgr.createServo(servoController, servoConf.getPort());
      map.servo.put(servoConf.getName(), s);
    } else if (servoConf.getType() == BuiltInConfigurationType.CONTINUOUS_ROTATION_SERVO) {
      CRServo s = deviceMgr.createCRServo(servoController, servoConf.getPort());
      map.crservo.put(servoConf.getName(), s);
    }
  }

  private void mapCoreInterfaceDeviceModule(HardwareMap map, DeviceManager deviceMgr, ControllerConfiguration ctrlConf) throws RobotCoreException, InterruptedException {
    if (!ctrlConf.isEnabled()) return;
    DeviceInterfaceModule deviceInterfaceModule = deviceMgr.createDeviceInterfaceModule(ctrlConf.getSerialNumber());
    map.deviceInterfaceModule.put(ctrlConf.getName(), deviceInterfaceModule);

    List<DeviceConfiguration> pwmDevices = ((DeviceInterfaceModuleConfiguration)ctrlConf).getPwmOutputs();
    buildDevices(pwmDevices, map, deviceMgr, deviceInterfaceModule);

    List<DeviceConfiguration> i2cDevices = ((DeviceInterfaceModuleConfiguration)ctrlConf).getI2cDevices();
    buildI2cDevices(i2cDevices, map, deviceMgr, deviceInterfaceModule);

    List<DeviceConfiguration> analogInputDevices = ((DeviceInterfaceModuleConfiguration)ctrlConf).getAnalogInputDevices();
    buildDevices(analogInputDevices, map, deviceMgr, deviceInterfaceModule);

    List<DeviceConfiguration> digitalDevices = ((DeviceInterfaceModuleConfiguration)ctrlConf).getDigitalDevices();
    buildDevices(digitalDevices, map, deviceMgr, deviceInterfaceModule);

    List<DeviceConfiguration> analogOutputDevices = ((DeviceInterfaceModuleConfiguration)ctrlConf).getAnalogOutputDevices();
    buildDevices(analogOutputDevices, map, deviceMgr, deviceInterfaceModule);
  }

  private void buildDevices(List<DeviceConfiguration> list, HardwareMap map, DeviceManager deviceMgr, DeviceInterfaceModule deviceInterfaceModule) {
    for (DeviceConfiguration deviceConfiguration : list) {
      ConfigurationType devType = deviceConfiguration.getType();
      if (devType==BuiltInConfigurationType.OPTICAL_DISTANCE_SENSOR) {
        mapOpticalDistanceSensor(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.ANALOG_INPUT) {
        mapAnalogInputDevice(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.TOUCH_SENSOR) {
        mapTouchSensor(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.DIGITAL_DEVICE) {
        mapDigitalDevice(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.PULSE_WIDTH_DEVICE) {
        mapPwmOutputDevice(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.ANALOG_OUTPUT) {
        mapAnalogOutputDevice(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.LED) {
        mapLED(map, deviceMgr, deviceInterfaceModule, deviceConfiguration);
      }
      else if (devType==BuiltInConfigurationType.NOTHING) {
        // nothing to do
      }
      else {
          RobotLog.w("Unexpected device type connected to Device Interface Module while parsing XML: " + devType.toString());
      }
    }
  }

  private void buildI2cDevices(List<DeviceConfiguration> list, HardwareMap map, DeviceManager deviceMgr, I2cController i2cController) {
    for (DeviceConfiguration deviceConfiguration : list) {
      ConfigurationType devType = deviceConfiguration.getType();
      if (devType==BuiltInConfigurationType.I2C_DEVICE) {
        mapI2cDevice(map, deviceMgr, i2cController, deviceConfiguration);
        continue;
      }
      if (devType==BuiltInConfigurationType.I2C_DEVICE_SYNCH) {
        mapI2cDeviceSynch(map, deviceMgr, i2cController, deviceConfiguration);
        continue;
      }
      if (devType==BuiltInConfigurationType.IR_SEEKER_V3) {
        mapIrSeekerV3Device(map, deviceMgr, i2cController, deviceConfiguration);
        continue;
      }
      if (devType==BuiltInConfigurationType.ADAFRUIT_COLOR_SENSOR) {
        mapAdafruitColorSensor(map, deviceMgr, i2cController, deviceConfiguration);
        continue;
      }
      if (devType==BuiltInConfigurationType.COLOR_SENSOR) {
        mapModernRoboticsColorSensor(map, deviceMgr, i2cController, deviceConfiguration);
        continue;
      }
      if (devType==BuiltInConfigurationType.GYRO) {
        mapModernRoboticsGyro(map, deviceMgr, i2cController, deviceConfiguration);
        continue;
      }
      if (devType==BuiltInConfigurationType.NOTHING) {
        // nothing to do
        continue;
      }
      if (devType.isI2cDevice()) {
        if (devType instanceof UserSensorType) {
          mapUserI2cDevice(map, deviceMgr, i2cController, deviceConfiguration);
          continue;
        }
      }
      RobotLog.w("Unexpected device type connected to I2c Controller while parsing XML: " + devType.toString());
    }
  }

  private void mapUsbLegacyModule(HardwareMap map, DeviceManager deviceMgr, ControllerConfiguration ctrlConf) throws RobotCoreException, InterruptedException {
    if (!ctrlConf.isEnabled()) return;
    LegacyModule legacyModule = deviceMgr.createUsbLegacyModule(ctrlConf.getSerialNumber());
    map.legacyModule.put(ctrlConf.getName(), legacyModule);

    for (DeviceConfiguration devConf : ctrlConf.getDevices()) {
      ConfigurationType devType = devConf.getType();
      if (devType==BuiltInConfigurationType.GYRO) {
        mapNxtGyroSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.COMPASS) {
        mapNxtCompassSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.IR_SEEKER) {
        mapNxtIrSeekerSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.LIGHT_SENSOR) {
        mapNxtLightSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.ACCELEROMETER) {
        mapNxtAccelerationSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.MOTOR_CONTROLLER) {
        mapNxtDcMotorController(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.SERVO_CONTROLLER) {
        mapNxtServoController(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.TOUCH_SENSOR) {
        mapNxtTouchSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.TOUCH_SENSOR_MULTIPLEXER) {
        mapNxtTouchSensorMultiplexer(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.ULTRASONIC_SENSOR) {
        mapSonarSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.COLOR_SENSOR) {
        mapNxtColorSensor(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.MATRIX_CONTROLLER) {
        mapMatrixController(map, deviceMgr, legacyModule, devConf);
      }
      else if (devType==BuiltInConfigurationType.NOTHING) {
        // nothing to do
      }
      else {
        RobotLog.w("Unexpected device type connected to Legacy Module while parsing XML: " + devType.toString());
      }
    }
  }

  private void mapIrSeekerV3Device(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    IrSeekerSensor irSeekerSensor = deviceMgr.createI2cIrSeekerSensorV3(i2cController, devConf.getPort());
    map.irSeekerSensor.put(devConf.getName(), irSeekerSensor);
  }

  private void mapDigitalDevice(HardwareMap map, DeviceManager deviceMgr, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    DigitalChannel digitalChannel = deviceMgr.createDigitalChannelDevice(deviceInterfaceModule, devConf.getPort());
    map.digitalChannel.put(devConf.getName(), digitalChannel);
  }

  private void mapTouchSensor(HardwareMap map, DeviceManager deviceMgr, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    TouchSensor touchSensor = deviceMgr.createDigitalTouchSensor(deviceInterfaceModule, devConf.getPort());
    map.touchSensor.put(devConf.getName(), touchSensor);
  }

  private void mapAnalogInputDevice(HardwareMap map, DeviceManager deviceMgr, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    AnalogInput analogInput = deviceMgr.createAnalogInputDevice(deviceInterfaceModule, devConf.getPort());
    map.analogInput.put(devConf.getName(), analogInput);
  }

  private void mapPwmOutputDevice(HardwareMap map, DeviceManager deviceMgr, PWMOutputController pwmOutputController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    PWMOutput pwmOutput = deviceMgr.createPwmOutputDevice(pwmOutputController, devConf.getPort());
    map.pwmOutput.put(devConf.getName(), pwmOutput);
  }

  private void mapI2cDevice(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    I2cDevice i2cDevice = deviceMgr.createI2cDevice(i2cController, devConf.getPort());
    map.i2cDevice.put(devConf.getName(), i2cDevice);
  }

  private void mapI2cDeviceSynch(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    I2cDevice i2cDevice = deviceMgr.createI2cDevice(i2cController, devConf.getPort());
    I2cDeviceSynch i2cDeviceSynch = new I2cDeviceSynchImpl(i2cDevice, true);
    map.i2cDeviceSynch.put(devConf.getName(), i2cDeviceSynch);
  }

  private void mapUserI2cDevice(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    UserSensorType userType = (UserSensorType)devConf.getType();
    HardwareDevice hardwareDevice = deviceMgr.createUserI2cDevice(i2cController, devConf.getPort(), userType);
    if (hardwareDevice != null) {
      // User-defined types don't live in a type-specific mapping, only in the overall one
      map.put(devConf.getName(), hardwareDevice);
    }
  }

  private void mapAnalogOutputDevice(HardwareMap map, DeviceManager deviceMgr, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    AnalogOutput analogOutput = deviceMgr.createAnalogOutputDevice(deviceInterfaceModule, devConf.getPort());
    map.analogOutput.put(devConf.getName(), analogOutput);
  }

  private void mapOpticalDistanceSensor(HardwareMap map, DeviceManager deviceMgr, DeviceInterfaceModule deviceInterfaceModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    OpticalDistanceSensor opticalDistanceSensor = deviceMgr.createAnalogOpticalDistanceSensor(deviceInterfaceModule, devConf.getPort());
    map.opticalDistanceSensor.put(devConf.getName(), opticalDistanceSensor);
  }

  private void mapNxtTouchSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    TouchSensor nxtTouchSensor = deviceMgr.createNxtTouchSensor(legacyModule, devConf.getPort());
    map.touchSensor.put(devConf.getName(), nxtTouchSensor);
  }

  private void mapNxtTouchSensorMultiplexer(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    TouchSensorMultiplexer nxtTouchSensorMultiplexer = deviceMgr.createNxtTouchSensorMultiplexer(legacyModule, devConf.getPort());
    map.touchSensorMultiplexer.put(devConf.getName(), nxtTouchSensorMultiplexer);
  }

  private void mapSonarSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    UltrasonicSensor sonarSensor = deviceMgr.createNxtUltrasonicSensor(legacyModule, devConf.getPort());
    map.ultrasonicSensor.put(devConf.getName(), sonarSensor);
  }

  private void mapNxtColorSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    ColorSensor colorSensor = deviceMgr.createNxtColorSensor(legacyModule, devConf.getPort());
    map.colorSensor.put(devConf.getName(), colorSensor);
  }

  private void mapNxtGyroSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    GyroSensor gyro = deviceMgr.createNxtGyroSensor(legacyModule, devConf.getPort());
    map.gyroSensor.put(devConf.getName(), gyro);
  }

  private void mapNxtCompassSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    CompassSensor compass = deviceMgr.createNxtCompassSensor(legacyModule, devConf.getPort());
    map.compassSensor.put(devConf.getName(), compass);
  }

  private void mapNxtIrSeekerSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    IrSeekerSensor irSeeker = deviceMgr.createNxtIrSeekerSensor(legacyModule, devConf.getPort());
    map.irSeekerSensor.put(devConf.getName(), irSeeker);
  }

  private void mapNxtLightSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    LightSensor light = deviceMgr.createNxtLightSensor(legacyModule, devConf.getPort());
    map.lightSensor.put(devConf.getName(), light);
  }

  private void mapNxtAccelerationSensor(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    AccelerationSensor accel = deviceMgr.createNxtAccelerationSensor(legacyModule, devConf.getPort());
    map.accelerationSensor.put(devConf.getName(), accel);
  }

  private void mapNxtDcMotorController(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration ctrlConf) {
    if (!ctrlConf.isEnabled()) return;
    HiTechnicNxtDcMotorController dcMotorController = (HiTechnicNxtDcMotorController)deviceMgr.createNxtDcMotorController(legacyModule, ctrlConf.getPort());
    map.dcMotorController.put(ctrlConf.getName(), dcMotorController);
    for (DeviceConfiguration motorConf : ((MotorControllerConfiguration) ctrlConf).getMotors()) {
      mapMotor(map, deviceMgr, motorConf, dcMotorController);
    }

    VoltageSensor voltageSensor = dcMotorController;
    map.voltageSensor.put(ctrlConf.getName(), voltageSensor);
  }

  private void mapNxtServoController(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    ServoController sc = deviceMgr.createNxtServoController(legacyModule, devConf.getPort());
    map.servoController.put(devConf.getName(), sc);
    for (DeviceConfiguration servoConf : ((ServoControllerConfiguration) devConf).getServos()) {
      mapServo(map, deviceMgr, servoConf, sc);
    }
  }

  private void mapMatrixController(HardwareMap map, DeviceManager deviceMgr, LegacyModule legacyModule, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    MatrixMasterController master = new MatrixMasterController((ModernRoboticsUsbLegacyModule)legacyModule, devConf.getPort());

    DcMotorController mc = new MatrixDcMotorController(master);
    map.dcMotorController.put(devConf.getName()+"Motor", mc);
    map.dcMotorController.put(devConf.getName(), mc);
    for (DeviceConfiguration motorConf : ((MatrixControllerConfiguration) devConf).getMotors()) {
      mapMotor(map, deviceMgr, motorConf, mc);
    }

    ServoController sc = new MatrixServoController(master);
    map.servoController.put(devConf.getName()+"Servo", sc);
    map.servoController.put(devConf.getName(), sc);
    for (DeviceConfiguration servoConf : ((MatrixControllerConfiguration) devConf).getServos()) {
      mapServo(map, deviceMgr, servoConf, sc);
    }
  }

  private void mapAdafruitColorSensor(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    ColorSensor colorSensor = deviceMgr.createAdafruitI2cColorSensor(i2cController, devConf.getPort());
    map.colorSensor.put(devConf.getName(), colorSensor);
  }

  private void mapLED(HardwareMap map, DeviceManager deviceMgr, DigitalChannelController digitalChannelController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    LED led = deviceMgr.createLED(digitalChannelController, devConf.getPort());
    map.led.put(devConf.getName(), led);
  }

  private void mapModernRoboticsColorSensor(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    ColorSensor colorSensor = deviceMgr.createModernRoboticsI2cColorSensor(i2cController, devConf.getPort());
    map.colorSensor.put(devConf.getName(), colorSensor);
  }

  private void mapModernRoboticsGyro(HardwareMap map, DeviceManager deviceMgr, I2cController i2cController, DeviceConfiguration devConf) {
    if (!devConf.isEnabled()) return;
    GyroSensor gyroSensor = deviceMgr.createModernRoboticsI2cGyroSensor(i2cController, devConf.getPort());
    map.gyroSensor.put(devConf.getName(), gyroSensor);
  }

  //------------------------------------------------------------------------------------------------
  // Serial number display name management
  //------------------------------------------------------------------------------------------------

  public static void noteSerialNumberType(Context context, SerialNumber serialNumber, String typeName) {
    synchronized (deviceDisplayNames) {
      deviceDisplayNames.put(serialNumber.toString(), String.format("%s [%s]", typeName, serialNumber.toString(context)));
    }
  }

  public static String getDeviceDisplayName(Context context, SerialNumber serialNumber) {
    synchronized (deviceDisplayNames) {
      String result = deviceDisplayNames.get(serialNumber.toString());
      if (result == null) {
        result = String.format(context.getString(R.string.deviceDisplayNameUnknownUSBDevice), serialNumber.toString(context));
      }
      return result;
    }
  }
}
