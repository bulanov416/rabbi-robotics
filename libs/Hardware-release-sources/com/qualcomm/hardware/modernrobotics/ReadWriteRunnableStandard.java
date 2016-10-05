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

package com.qualcomm.hardware.modernrobotics;

import android.content.Context;

import com.qualcomm.hardware.HardwareFactory;
import com.qualcomm.hardware.R;
import com.qualcomm.modernrobotics.ReadWriteRunnableUsbHandler;
import com.qualcomm.robotcore.exception.FTDeviceClosedException;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.usb.RobotUsbModule;
import com.qualcomm.robotcore.hardware.usb.RobotUsbDevice;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.SerialNumber;
import com.qualcomm.robotcore.util.TypeConversion;
import com.qualcomm.robotcore.util.ThreadPool;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

/**
 * Main loop that copies the read/write buffer from/to the device
 * <p>
 * Helper class for Modern Robotics USB Device
 */
public class ReadWriteRunnableStandard implements ReadWriteRunnable {

  //------------------------------------------------------------------------------------------------
  // State
  //------------------------------------------------------------------------------------------------

  protected final byte[] localDeviceReadCache = new byte[MAX_BUFFER_SIZE];
  protected final byte[] localDeviceWriteCache = new byte[MAX_BUFFER_SIZE];

  protected Map<Integer, ReadWriteRunnableSegment> segments = new HashMap<Integer, ReadWriteRunnableSegment>();
  protected ConcurrentLinkedQueue<Integer> segmentReadQueue = new ConcurrentLinkedQueue<Integer>();
  protected ConcurrentLinkedQueue<Integer> segmentWriteQueue = new ConcurrentLinkedQueue<Integer>();

  protected final Context               context;
  protected final SerialNumber          serialNumber;
  protected RobotUsbDevice              robotUsbDevice;
  protected ReadWriteRunnableUsbHandler usbHandler;

  protected int startAddress;
  protected int monitorLength;

  protected          CountDownLatch runningInterlock  = new CountDownLatch(1);
  protected volatile boolean        running           = false;
  protected volatile boolean        shutdownAbnormally = false;
  protected volatile boolean        shutdownComplete  = false;
  private   volatile boolean        writeNeeded       = false;
  protected final    Object         acceptingWritesLock = new Object();
  protected volatile boolean        acceptingWrites   = false;

  protected         Callback        callback;
  protected         RobotUsbModule  owner;

  protected final boolean DEBUG_LOGGING;

  //------------------------------------------------------------------------------------------------
  // Construction
  //------------------------------------------------------------------------------------------------

  /**
   * Constructor
   * @param serialNumber USB serial number
   * @param device FTDI device
   * @param monitorLength length of memory map to continually monitor
   * @param startAddress starting address of changing buffer values.
   */
  public ReadWriteRunnableStandard(Context context, SerialNumber serialNumber, RobotUsbDevice device, int monitorLength, int startAddress, boolean debug) {
    this.context      = context;
    this.serialNumber = serialNumber;
    this.startAddress = startAddress;
    this.monitorLength = monitorLength;
    this.DEBUG_LOGGING = debug;
    this.callback = new EmptyCallback();
    this.owner    = null;
    this.robotUsbDevice = device;
    this.usbHandler = new ReadWriteRunnableUsbHandler(device);
  }

  public void setCallback(Callback callback) {
    this.callback = callback;
  }

  @Override
  public void setOwner(RobotUsbModule owner) {
    this.owner = owner;
  }

  @Override
  public RobotUsbModule getOwner() {
    return this.owner;
  }

//------------------------------------------------------------------------------------------------
  // Operations
  //------------------------------------------------------------------------------------------------

  /**
   * This method should block until the read / write loop has completed a read, and is waiting to write
   */
  @Override
  public void blockUntilReady() throws RobotCoreException, InterruptedException {
    // Nothing to do
    }

  /**
   * This method let's the read / write loop know it's ok to continue
   */
  @Override
  public void startBlockingWork() {
    // take no action; this device fakes being sync'd
  }

  @Override
  public boolean writeNeeded() {
    return writeNeeded;
  }

  @Override
  public void resetWriteNeeded() {
    writeNeeded = false;
  }

  /**
   * Write to device
   *
   * @param address address to write
   * @param data data to write
   */
  @Override
  public void write(int address, byte[] data) {
    synchronized (this.acceptingWritesLock) {
      if (this.acceptingWrites) {
        synchronized (localDeviceWriteCache) {
          System.arraycopy(data, 0, localDeviceWriteCache, address, data.length);
          writeNeeded = true;
        }
      }
    }
  }

  /**
   * Read from the device write cache
   *
   * @param address address of byte to read
   * @param size number of bytes to read
   * @return byte array
   */
  @Override
  public byte[] readFromWriteCache(int address, int size) {
    synchronized (localDeviceWriteCache) {
      return Arrays.copyOfRange(localDeviceWriteCache, address, address + size);
    }
  }

  /**
   * Read from device
   *
   * @param address address to read
   * @param size number of bytes to read
   */
  @Override
  public byte[] read(int address, int size) {
    synchronized (localDeviceReadCache) {
      return Arrays.copyOfRange(localDeviceReadCache, address, address + size);
    }
  }

  @Override
  public void executeUsing(ExecutorService service) {
    // avoid concurrent close() attempts
    synchronized (this) {
      service.execute(this);
      this.awaitRunning();  // wait until the run() has set the 'running' variable
    }
  }

  /**
   * Close this read-write-runnable
   */
  @Override
  public void close() {

    synchronized (this) {
      // There's nothing to do if we haven't yet been started.
      if (running) {

        // Unstick FT_Device.read if it's sitting there in a timeout
        this.robotUsbDevice.requestInterrupt();

        // tell current loop that we want it to exit
        running = false;

        try {
          // wait for the current loop to finish reading
          blockUntilReady();

          // allow one final write
          startBlockingWork();
        } catch (InterruptedException e) {
          // if we receive an interrupt, close early
          RobotLog.w("Exception while closing USB device: " + e.getMessage());
        } catch (RobotCoreException e) {
          // if we receive an interrupt, close early
          RobotLog.w("Exception while closing USB device: " + e.getMessage());
        }

        while (!shutdownComplete) Thread.yield(); // busy wait for shutdown
      }
    }
  }

  public ReadWriteRunnableSegment createSegment(int key, int address, int size) {
    ReadWriteRunnableSegment segment = new ReadWriteRunnableSegment(address, size);
    segments.put(key, segment);
    return segment;
  }

  public void destroySegment(int key) {
    segments.remove(key);
  }

  public ReadWriteRunnableSegment getSegment(int key) {
    return segments.get(key);
  }

  public void queueSegmentRead(int key) {
    queueIfNotAlreadyQueued(key, segmentReadQueue);
  }

  public void queueSegmentWrite(int key) {
        synchronized (this.acceptingWritesLock) {
          if (acceptingWrites) {
        queueIfNotAlreadyQueued(key, segmentWriteQueue);
      }
    }
  }

  protected void awaitRunning() {
  // We *must* wait, by jove, even in the face of interrupts, or otherwise it won't
  // be possible to reliably close().
    try {
      // Wait in the good way
      runningInterlock.await();
    } catch (InterruptedException ignored) {
      // Can't wait that way; spin instead until run() advances to the point we expect
      while (runningInterlock.getCount() != 0)
        Thread.yield();
      // Ok, NOW we can propagate the interrupt
      Thread.currentThread().interrupt();
    }
  }

/**
   * Main read / write loop
   */
  @Override
  public void run() {

    // Paranoia: avoid re-execution, as we weren't designed for that
    if (shutdownComplete)
      return;

    ThreadPool.logThreadLifeCycle(String.format("r/w loop: %s", HardwareFactory.getDeviceDisplayName(context, serialNumber)), new Runnable() { @Override public void run() {

    // read the entire buffer at least one time.
    boolean isFirstRun = true;
    int address = 0x00; // Where in the local device cache to begin writing.
    byte[] buffer = new byte[(monitorLength + startAddress)]; // read in the entire buffer once.

    ElapsedTime timer = new ElapsedTime();
    String timerString = "Device " + serialNumber.toString();

    // Tell those awaiting our startup that we are up and running
    running = true;

    try {
      callback.startupComplete();
    } catch (InterruptedException e) {
      // shutdown, in the normal way, soon!
      running = false;
    }
    runningInterlock.countDown();

    // Note: we must avoid exceptions up to this point, as it is essential that when we
    // are started up. See executeUsing().

    // Ok: let's ... go ... fly, a kite ... :-)

    try {
      usbHandler.purge(RobotUsbDevice.Channel.RX);

      while (running) {

        if (DEBUG_LOGGING) {
          timer.log(timerString);
          timer.reset();
        }

        /*
         * read
         */
        try {
          // read the main buffer
          usbHandler.read(address, buffer);

          // read any segments
          while (!segmentReadQueue.isEmpty()) {
            ReadWriteRunnableSegment segment = segments.get(segmentReadQueue.remove());
            byte[] readBuffer = new byte[segment.getReadBuffer().length];
            usbHandler.read(segment.getAddress(), readBuffer);
            try {
              segment.getReadLock().lock();
              System.arraycopy(readBuffer, 0, segment.getReadBuffer(), 0, segment.getReadBuffer().length);
            } finally {
              segment.getReadLock().unlock();
            }
          }
        } catch (FTDeviceClosedException e) {
          throw e;
        } catch (RobotCoreException e) {
          RobotLog.w(String.format("could not read %s: %s", HardwareFactory.getDeviceDisplayName(context, serialNumber), e.getMessage()));
        }
        synchronized (localDeviceReadCache) {
          System.arraycopy(buffer, 0, localDeviceReadCache, address, buffer.length);
        }

        if (DEBUG_LOGGING) dumpBuffers("read", localDeviceReadCache);
        callback.readComplete();

        /*
         * Wait for Sync'd Events
         */
        waitForSyncdEvents();

        /*
         * Handle first run special cases
         */
        if (isFirstRun) {
          // access only the changing part of the buffer from now on.
          address = startAddress;
          buffer = new byte[(monitorLength)];
          isFirstRun = false;
        }

        /*
         * write
         */
        synchronized (localDeviceWriteCache) {
          System.arraycopy(localDeviceWriteCache, address, buffer, 0, buffer.length);
        }

        try {
          // write the main buffer
          if (writeNeeded()) {
            usbHandler.write(address, buffer);
            resetWriteNeeded();
          }

          // write any segments
          while (!segmentWriteQueue.isEmpty()) {
            ReadWriteRunnableSegment segment = segments.get(segmentWriteQueue.remove());
            byte[] writeBuffer;
            try {
              segment.getWriteLock().lock();
              writeBuffer = Arrays.copyOf(segment.getWriteBuffer(), segment.getWriteBuffer().length);
            } finally {
              segment.getWriteLock().unlock();
            }
            usbHandler.write(segment.getAddress(), writeBuffer);
          }
        } catch (FTDeviceClosedException e) {
          throw e;
        } catch(RobotCoreException e) {
          RobotLog.w(String.format("could not write to %s: %s", HardwareFactory.getDeviceDisplayName(context, serialNumber), e.getMessage()));
        }

        if (DEBUG_LOGGING) dumpBuffers("write", localDeviceWriteCache);
        callback.writeComplete();

        usbHandler.throwIfUsbErrorCountIsTooHigh();
      }
    } catch (NullPointerException e) {
      RobotLog.w(String.format("could not write to %s: FTDI Null Pointer Exception", HardwareFactory.getDeviceDisplayName(context, serialNumber)));
      RobotLog.logStacktrace(e);
      setOwnerWarningMessage(context.getString(R.string.warningProblemCommunicatingWithUSBDevice), HardwareFactory.getDeviceDisplayName(context, serialNumber));
      shutdownAbnormally = true;

    } catch (InterruptedException e) {
      RobotLog.w(String.format("thread interrupt: could not write to %s", HardwareFactory.getDeviceDisplayName(context, serialNumber)));
      shutdownAbnormally = true;

    } catch (RobotCoreException e) {
      RobotLog.w(e.getMessage());
      setOwnerWarningMessage(context.getString(R.string.warningProblemCommunicatingWithUSBDevice), HardwareFactory.getDeviceDisplayName(context, serialNumber));
      shutdownAbnormally = true;

    } finally {
      usbHandler.close();
      running = false;
      try {
        callback.shutdownComplete();
      } catch (InterruptedException e) {
        // ignore: we're about to terminate this thread anyway
      }
      shutdownComplete = true;
    }
    }});
  }

  void setOwnerWarningMessage(String format, Object...args) {
    String message = String.format(format, args);

    if (this.owner != null && (this.owner instanceof GlobalWarningSource)) {
      ((GlobalWarningSource)this.owner).setGlobalWarning(message);
    } else {
      RobotLog.setGlobalWarningMessage(message);
    }
  }

  @Override
  public boolean hasShutdownAbnormally() {
    return shutdownAbnormally;
    }

  boolean hasPendingWrites() {
    return writeNeeded || !segmentWriteQueue.isEmpty();
  }

  @Override
  public void drainPendingWrites() {
    while (running && hasPendingWrites()) {
      // make sure we're not stuck in waitForSyncdEvents()
      startBlockingWork();
      Thread.yield();
    }
  }

  @Override public void setAcceptingWrites(boolean acceptingWrites) {
    synchronized (this.acceptingWritesLock) {
      this.acceptingWrites = acceptingWrites;
    }
  }

  @Override public boolean getAcceptingWrites() {
    return this.acceptingWrites;
    }

  protected void waitForSyncdEvents() throws RobotCoreException, InterruptedException {
    // by default take no action
  }

  protected void dumpBuffers(String name, byte[] byteArray) {
    RobotLog.v("Dumping " + name + " buffers for " + serialNumber);

    /*
     * We want to print the contents of the local device cache, but not all of it. Only the
     * bytes up to the device buffer length matter.
     */

    StringBuilder s = new StringBuilder(MAX_BUFFER_SIZE * 4);
    for (int i = 0; i < startAddress + monitorLength; i++) {
      s.append(String.format(" %02x", TypeConversion.unsignedByteToInt(byteArray[i])));
      if ((i + 1) % 16 == 0) s.append("\n");
    }

    RobotLog.v(s.toString());
  }

  protected void queueIfNotAlreadyQueued(int key, ConcurrentLinkedQueue<Integer> queue) {
    if (!queue.contains(key)) {
      queue.add(key);
    }
  }
}

