/* Description and License
 * MCBMini is a complete, open-source, flexible and scalable 
 * motor control scheme with board designs, firmware and host 
 * software. 
 * This is the host software for MCBMini called MCBMiniServer
 * The MCBMini project can be downloaded from:
 * http://code.google.com/p/mcbmini/ 
 *
 * (c) Sigurdur Orn Adalgeirsson (siggi@alum.mit.edu)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2 of the License.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 */
 
package mcbmini;

import gnu.io.SerialPort;

import ioio.lib.api.IOIO;
import ioio.lib.api.IOIOFactory;
import ioio.lib.api.Uart;
import ioio.lib.api.exception.ConnectionLostException;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Properties;

import mcbmini.serial.SerialEventHandler;
import mcbmini.serial.iSerial;

/**
 * @author asetapen
 * @date Jul 18, 2011
 */
public class AndroidIOIOPSerial implements iSerial {

	private int ioio_rx_pin;
	private int ioio_tx_pin;

	protected IOIO ioio_;
	protected Object ioio_lock;

	public static final int IOIO_BAUD_RATE = 115200;

	public int rate;
	public int parity;
	public int databits;
	public float stopbits;

	private boolean initialized;

	// read buffer and streams
	public InputStream input;
	public OutputStream output;
	private ArrayList<SerialEventHandler> event_handlers;

	byte buffer[] = new byte[32768];
	int bufferIndex;
	int bufferLast;

	// defaults
	static int drate = 115200;
	static char dparity = SerialPort.PARITY_NONE;
	static int ddatabits = 8;
	static float dstopbits = 1;

	public void setProperties(Properties props) {
		rate = Integer.parseInt(props.getProperty("serial.rate", drate + ""));
		parity = props.getProperty("serial.parity", dparity + "").charAt(0);
		databits = Integer.parseInt(props.getProperty("seryial.databits", ddatabits + ""));
		stopbits = new Float(props.getProperty("serial.stopbits", dstopbits + "")).floatValue();
	}

	public AndroidIOIOPSerial(int ioio_rx_pin, int ioio_tx_pin) {
		this.ioio_rx_pin = ioio_rx_pin;
		this.ioio_tx_pin = ioio_tx_pin;
		initialized = false;
		ioio_lock = new Object();
		Thread ioioThread = new IOIOCommThread(this);
		ioioThread.start();

		rate = drate;
		parity = dparity;
		databits = ddatabits;
		stopbits = dstopbits;

		event_handlers = new ArrayList<SerialEventHandler>();

		// Add shutdown hooks
		Runtime.getRuntime().addShutdownHook(new Thread() {
			public void run() {

			}
		});
	}

	public IOIO getIOIO(){
		return ioio_;
	}

	public Object getIOIOThreadLock(){
		return ioio_lock;
	}

	public void addSerialEventHandler(SerialEventHandler handler){
		synchronized (event_handlers) {
			event_handlers.add(handler);
		}
	}

	public void removeSerialEventHandler(SerialEventHandler handler){
		synchronized (event_handlers) {
			event_handlers.remove(handler);
		}
	}

	/**
	 * Returns the number of bytes that have been read from serial
	 * and are waiting to be dealt with by the user.
	 */
	public int available() {
		return (bufferLast - bufferIndex);
	}

	/**
	 * Ignore all the bytes read so far and empty the buffer.
	 */
	public void clear() {
		bufferLast = 0;
		bufferIndex = 0;
	}

	/**
	 * Returns a number between 0 and 255 for the next byte that's
	 * waiting in the buffer.
	 * Returns -1 if there was no byte (although the user should
	 * first check available() to see if things are ready to avoid this)
	 */
	public byte readByte() {
		if (bufferIndex == bufferLast) return -1;

		synchronized (buffer) {
			byte outgoing = buffer[bufferIndex++];
			if (bufferIndex == bufferLast) {  // rewind
				bufferIndex = 0;
				bufferLast = 0;
			}
			return outgoing;
		}
	}

	public void write(byte bytes[]) throws IOException {
		write(bytes, 0, bytes.length);
	}

	public void write(byte bytes[], int offs, int len) throws IOException {
		output.write(bytes, offs, len);
		//		output.flush();   // hmm, not sure if a good idea
	}

	/**
	 * General error reporting
	 */
	static public void errorMessage(String where, Throwable e) {
		e.printStackTrace();
		throw new RuntimeException("Error inside IOIOSerial." + where + "()");
	}

	/**
	 * An abstract class, which facilitates a thread dedicated for IOIO
	 * communication.
	 */
	protected class IOIOCommThread extends Thread {
		/** Subclasses should use this field for controlling the IOIO. */
		private boolean abort_ = false;
		protected Uart uart;
		protected AndroidIOIOPSerial parent;

		public IOIOCommThread(AndroidIOIOPSerial parent){
			this.parent = parent;
		}

		/** Not relevant to subclasses. */
		//@Override
		public final void run() {
			super.run();
			while (true) {
				try {
					synchronized (ioio_lock) {
						if (abort_) {
							break;
						}
						ioio_ = IOIOFactory.create();
					}
					ioio_.waitForConnect();
					setup();
					synchronized (ioio_lock) {
						initialized = true;
					}
					while (true) {
						loop();
					}
				} catch (ConnectionLostException e) {
					if (abort_) {
						break;
					}
				} catch (Exception e) {
					//					Log.e("AbstractIOIOActivity", "Unexpected exception caught", e);
					ioio_.disconnect();
					break;
				} finally {
					try {
						if( ioio_ != null ) ioio_.waitForDisconnect();
					} catch (InterruptedException e) {
					}
					break;
				}
			}
		}

		/**
		 * Subclasses should override this method for performing operations to
		 * be done once as soon as IOIO communication is established. Typically,
		 * this will include opening pins and modules using the openXXX()
		 * methods of the {@link #ioio_} field.
		 */
		protected void setup() throws ConnectionLostException {
			uart = ioio_.openUart(ioio_rx_pin, ioio_tx_pin, IOIO_BAUD_RATE, Uart.Parity.NONE, Uart.StopBits.ONE);
			input = uart.getInputStream();
			output = uart.getOutputStream();
		}

		/**
		 * Subclasses should override this method for performing operations to
		 * be done repetitively as long as IOIO communication persists.
		 * Typically, this will be the main logic of the application, processing
		 * inputs and producing outputs.
		 */
		protected void loop() throws ConnectionLostException {
			try {
				while (input.available() > 0) {
					synchronized (buffer) {
						if (bufferLast == buffer.length) {
							byte temp[] = new byte[bufferLast << 1];
							System.arraycopy(buffer, 0, temp, 0, bufferLast);
							buffer = temp;
						}
						buffer[bufferLast++] = (byte) input.read();
					}
					// Here we notify all handlers
					synchronized (event_handlers) {
						for (SerialEventHandler handler : event_handlers) {
							handler.handleSerialDataAvailableEvent(parent);
						}
					}
				}
			} catch (IOException e) {
				errorMessage("ReaderThread.run()", e);
			}

			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {
				errorMessage("ReaderThread.run()", e);
			}
		}

		/** Not relevant to subclasses. */
		public synchronized final void abort() {
			abort_ = true;
			if (ioio_ != null) {
				ioio_.disconnect();
			}
		}
	}

//	//@Override
	public boolean isInitialized() {
		synchronized (ioio_lock) {
			return initialized;
		}
	}
}