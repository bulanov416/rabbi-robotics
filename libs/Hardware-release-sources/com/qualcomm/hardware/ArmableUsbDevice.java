
package com.qualcomm.hardware;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.EventLoopManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.hardware.usb.RobotUsbModule;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.WeakReferenceSet;

import junit.framework.Assert;

import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;

/**
 * Created by bob on 2016-03-11.
 */
public abstract class ArmableUsbDevice implements RobotUsbModule, GlobalWarningSource
    {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected final Context                 context;
    protected final SerialNumber            serialNumber;
    protected final EventLoopManager        eventLoopManager;
    protected final OpenRobotUsbDevice      openRobotUsbDevice;
    protected       RobotUsbDevice          robotUsbDevice;

    protected           ARMINGSTATE             armingState;
    protected final     Object                  armingLock                 = new Object();
    protected final  WeakReferenceSet<Callback> registeredCallbacks        = new WeakReferenceSet<>();
    protected final     Object                  warningMessageLock         = new Object();
    protected           int                     warningMessageSuppressionCount;
    protected           String                  warningMessage;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public interface OpenRobotUsbDevice
        {
        RobotUsbDevice open() throws RobotCoreException, InterruptedException;
        }

    public ArmableUsbDevice(Context context, SerialNumber serialNumber, EventLoopManager manager, OpenRobotUsbDevice openRobotUsbDevice)
        {
        this.context              = context;
        this.serialNumber         = serialNumber;
        this.eventLoopManager     = manager;
        this.openRobotUsbDevice   = openRobotUsbDevice;
        this.robotUsbDevice       = null;
        this.armingState          = ARMINGSTATE.DISARMED;
        this.warningMessageSuppressionCount = 0;
        this.warningMessage       = "";
        }

    protected void finishConstruction()
        {
        RobotLog.registerGlobalWarningSource(this);
        }

    //----------------------------------------------------------------------------------------------
    // Accessors
    //----------------------------------------------------------------------------------------------

    public Context getContext()
        {
        return this.context;
        }

    //----------------------------------------------------------------------------------------------
    // RobotUsbModule
    //----------------------------------------------------------------------------------------------

    @Override
    public SerialNumber getSerialNumber()
        {
        return serialNumber;
        }

    @Override
    public void registerCallback(Callback callback)
        {
        registeredCallbacks.add(callback);
        }

    @Override
    public void unregisterCallback(Callback callback)
        {
        registeredCallbacks.remove(callback);
        }

    //----------------------------------------------------------------------------------------------
    // GlobalWarningSource
    //----------------------------------------------------------------------------------------------

    @Override
    public String getGlobalWarning()
        {
        synchronized (this.warningMessageLock)
            {
            return this.warningMessageSuppressionCount > 0 ? "" : this.warningMessage;
            }
        }

    @Override
    public void clearGlobalWarning()
        {
        synchronized (this.warningMessageLock)
            {
            this.warningMessage = "";
            this.warningMessageSuppressionCount = 0;
            }
        }

    @Override
    public void suppressGlobalWarning(boolean suppress)
        {
        synchronized (this.warningMessageLock)
            {
            if (suppress)
                this.warningMessageSuppressionCount++;
            else
                this.warningMessageSuppressionCount--;
            }
        }

    @Override
    public boolean setGlobalWarning(String warning)
        {
        // We need to lock so we can atomically test-and-set. We can't lock on us, the whole
        // object, as that will cause deadlocks due to interactions with operational synchronized
        // methods (in subclasses). Thus, we need to use a leaf-level lock.
        synchronized (this.warningMessageLock)
            {
            if (warning != null && warningMessage.isEmpty())
                {
                this.warningMessage = warning;
                return true;
                }
            return false;
            }
        }

    //----------------------------------------------------------------------------------------------
    // Internal arming and disarming
    //----------------------------------------------------------------------------------------------

    protected void armDevice() throws RobotCoreException, InterruptedException
        {
        synchronized (armingLock)
            {
            // An arming attempt clears any extant warning
            this.warningMessage = "";

            Assert.assertTrue(this.robotUsbDevice == null);
            RobotUsbDevice device = null;
            try
                {
                // Open the USB device
                device = this.openRobotUsbDevice.open();
                // Arm using that device
                armDevice(device);
                }
            catch (RobotCoreException e)
                {
                if (device != null) device.close();
                setGlobalWarning(String.format(context.getString(R.string.warningUnableToOpen), HardwareFactory.getDeviceDisplayName(context, serialNumber)));
                throw e;
                }
            catch (NullPointerException e)
                {
                // NullPointerException(s) are, annoyingly (vs some non-runtime exception), thrown by the FTDI
                // layer on abnormal termination. Here, and elsewhere in this class, we catch those in order to
                // robustly recover from what is just a USB read or write error.
                if (device != null) device.close();
                setGlobalWarning(String.format(context.getString(R.string.warningUnableToOpen), HardwareFactory.getDeviceDisplayName(context, serialNumber)));
                throw e;
                }
            catch (InterruptedException e)
                {
                if (device != null) device.close();
                throw e;
                }
            }
        }

    protected void pretendDevice() throws RobotCoreException, InterruptedException
        {
        synchronized (armingLock)
            {
            // Make a pretend device
            RobotUsbDevice device = this.getPretendDevice(this.serialNumber);
            // Arm using that device
            armDevice(device);
            }
        }

    protected RobotUsbDevice getPretendDevice(SerialNumber serialNumber)
        {
        return null;
        }

    protected abstract void armDevice(RobotUsbDevice device) throws RobotCoreException, InterruptedException;

    protected abstract void disarmDevice() throws InterruptedException;

    //----------------------------------------------------------------------------------------------
    // Arming and disarming
    //----------------------------------------------------------------------------------------------

    @Override
    public ARMINGSTATE getArmingState()
        {
        return this.armingState;
        }

    protected ARMINGSTATE setArmingState(ARMINGSTATE state)
        {
        ARMINGSTATE prev = this.armingState;
        this.armingState = state;
        for (Callback callback : registeredCallbacks)
            {
            callback.onModuleStateChange(this, state);
            }
        return prev;
        }

    protected boolean isArmed()
        {
        return this.armingState == ARMINGSTATE.ARMED;
        }
    protected boolean isArmedOrArming()
        {
        return this.armingState == ARMINGSTATE.ARMED || this.armingState == ARMINGSTATE.TO_ARMED;
        }

    protected boolean isPretending()
        {
        return this.armingState == ARMINGSTATE.PRETENDING;
        }


    @Override
    public void arm() throws RobotCoreException, InterruptedException
        {
        synchronized (armingLock)
            {
            try
                {
                switch (this.armingState)
                    {
                    case ARMED:
                        return;
                    case DISARMED:
                        ARMINGSTATE prev = setArmingState(ARMINGSTATE.TO_ARMED);
                        try {
                            this.doArm();
                            setArmingState(ARMINGSTATE.ARMED);
                            }
                        catch (Exception e)
                            {
                            setArmingState(prev);
                            throw e;
                            }
                        break;
                    default:
                        throw new RobotCoreException("illegal state: can't arm() from state %s", this.armingState.toString());
                    }
                }
            catch (RobotCoreException e)
                {
                disarm();
                throw e;
                }
            catch (InterruptedException e)
                {
                disarm();
                throw e;
                }
            catch (NullPointerException e)
                {
                disarm();
                throw e;
                }
            }
        }

    /** intended as subclass hook */
    protected void doArm() throws RobotCoreException, InterruptedException
        {
        this.armDevice();
        }

    @Override
    public void pretend() throws RobotCoreException, InterruptedException
        {
        synchronized (armingLock)
            {
            try
                {
                switch (this.armingState)
                    {
                    case PRETENDING:
                        return;
                    case DISARMED:
                        ARMINGSTATE prev = setArmingState(ARMINGSTATE.TO_PRETENDING);
                        try {
                            this.doPretend();
                            setArmingState(ARMINGSTATE.PRETENDING);
                            }
                        catch (Exception e)
                            {
                            setArmingState(prev);
                            throw e;
                            }
                        break;
                    default:
                        throw new RobotCoreException("illegal state: can't pretend() from state %s", this.armingState.toString());
                    }
                }
            catch (RobotCoreException|RuntimeException|InterruptedException e)
                {
                disarm();
                throw e;
                }
            }
        }

    @Override
    public void armOrPretend() throws RobotCoreException, InterruptedException
        {
        synchronized (armingLock)
            {
            try
                {
                arm();
                }
            catch (RobotCoreException|RuntimeException e)
                {
                pretend();
                }
            catch (InterruptedException e)
                {
                pretend();
                Thread.currentThread().interrupt();
                }
            }
        }

    /** intended for subclasses to override */
    protected void doPretend() throws RobotCoreException, InterruptedException
        {
        this.pretendDevice();
        }

    @Override
    public void disarm() throws RobotCoreException, InterruptedException
        {
        synchronized (this.armingLock)
            {
            switch (this.armingState)
                {
                case DISARMED:
                    return;
                case TO_ARMED:
                case ARMED:
                case TO_PRETENDING:
                case PRETENDING:
                    ARMINGSTATE prev = setArmingState(ARMINGSTATE.TO_DISARMED);
                    try {
                        this.doDisarm();
                        setArmingState(ARMINGSTATE.DISARMED);
                        }
                    catch (Exception e)
                        {
                        setArmingState(prev);
                        throw e;
                        }
                    break;
                default:
                    throw new RobotCoreException("illegal state: can't disarm() from state %s", this.armingState.toString());
                }
            }
        }

    /** intended as subclass hook */
    protected void doDisarm() throws RobotCoreException, InterruptedException
        {
        this.disarmDevice();
        }


    @Override
    public void close()
        {
        synchronized (this.armingLock)
            {
            try
                {
                switch (this.armingState)
                    {
                    case CLOSED:
                        return;
                    case ARMED:
                        this.doCloseFromArmed();
                        break;
                    default:
                        this.doCloseFromOther();
                        break;
                    }
                }
            catch (InterruptedException e)
                {
                Thread.currentThread().interrupt();
                }
            catch (RobotCoreException|RuntimeException e)
                {
                // Eat the exception: we are close()ing now, after all
                }
            finally
                {
                // In all cases, note us as closed on exit. We tried to close down once; even
                // if we got an error, there's nothing more we can reasonably do!
                setArmingState(ARMINGSTATE.CLOSED);
                this.warningMessage = "";
                }
            }
        }

    /** intended as subclass hook */
    protected void doCloseFromArmed() throws RobotCoreException, InterruptedException
        {
        this.disarm();
        }

    /** intended as subclass hook */
    protected void doCloseFromOther() throws RobotCoreException, InterruptedException
        {
        this.disarm();
        }

    }
