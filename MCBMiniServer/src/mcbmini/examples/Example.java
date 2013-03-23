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

package mcbmini.examples;

import java.io.IOException;

import mcbmini.AndroidIOIOMCBMiniServer;
import mcbmini.DebugMCBMiniServer;
import mcbmini.MCBMiniBoard;
import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.Command;
import mcbmini.MCBMiniServer;
import mcbmini.MCBMiniServer.MCBMiniBoardDisabledHandler;
import mcbmini.MCBMiniServer.MCBMiniResponseHandler;
import mcbmini.serial.MCBMiniNativeLoader;
import mcbmini.utils.Log;
import mcbmini.utils.XMLUtils;
import mcbmini.utils.XMLUtils.XMLResults;

/**
 * @author siggi
 * @date Jul 18, 2012
 */
public class Example {

	static final boolean DEBUG = true;

	public static void main(String[] args) {

		/*
		 * [OPTIONAL]
		 * We can choose to manually specify a location for the rxtx native library for our platform
		 * If we don't, the library will be searched for in a lib folder next to the executable that is running. Folder structure should be the following:
		 * ./lib/{linux,osx,windows}/{32bit,64bit}/{lib}rxtxSerial.{so,jnilib,dll}
		 */
		System.setProperty("mcbmini.rxtx.library.path", "/bla/librxtxSerial.jnilib");

		/*
		 * Read the xml file whose path was passed in as a cmd line argument
		 */
		args = new String[]{"test.xml"};
		XMLResults results = null;
		try {
			results = XMLUtils.parseMCBMiniConfigFile( args[0] );
		} catch (Exception e1) {
			Log.println("Can't parse input file: "+e1.getMessage(), true);
			System.exit(0);
		}


		/*
		 * Initialize the server with the board list and serial port name from the xml file
		 */
		MCBMiniServer server = null;
		try {
			// This allows us to not have a connected stack, for debugging purposes
			if( DEBUG ) server = new DebugMCBMiniServer(results.boards);
			else server = new MCBMiniServer(results.port_name, results.boards);
//			server = new AndroidIOIOMCBMiniServer(boards, ioio_rx_pin, ioio_tx_pin)
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		/*
		 * Enable the board
		 * We could also change any other parameters if we wanted to
		 */
		MCBMiniBoard board = server.getBoards().get(0);
		board.setEnabled(Channel.A, true);

		/*
		 * Register a handler for disable events (boards can be disabled on timeouts or fault conditions like overheating)
		 */
		server.addBoardDisabledEventHandler(new MCBMiniBoardDisabledHandler() {
			@Override
			public void handleBoardDisableEvent(MCBMiniBoard board, Channel ch) {
				Log.println("Board notified disable event: "+board.getId()+": "+ch);
			}
		});


		/*
		 * Create a loop that loops at 50Hz forever
		 */
		int index = 0;
		while(true){
			index++;
			// Update the server, this is important and has to be done in an update loop of the application main thread
			server.update();

			// Create the sinusoidal signal
			int sin = (int)( 512 + 500 * Math.sin( (float)index/20f ) );

			// Set the target position of the motor to the sinusoidal signal
			board.setTargetTick(Channel.A, sin);

			// Print out the actual tick position for the motor
			Log.println("Actual tick: " + board.getActualTick(Channel.A) );

			// Every now and then make a request for a parameter just for fun
			if( index % 200 == 0 ){
				server.sendRequestForResponse(board, Channel.A, Command.MOTOR_CURRENT, new MCBMiniResponseHandler() {
					@Override
					public void handleResponse(MCBMiniBoard board, Channel channel, Command command, int value) {
						Log.println("Received response to request: "+command+": "+value);
					}
				});
			}

			/*
			 * Every now and then we print out the update rates of the system (the 2nd one is more interesting, it is the response frequency of the boards
			 * if it is ever very different from our internal one then we might have a comm problem)
			 */
			if( index % 100 == 0 ){
				float[] fps = server.getUpdateRates(null);
				Log.println("Internal update rate: "+fps[0]+" Hz");
				Log.println("Board cycle update rate: "+fps[1]+" Hz");
			}

			// Sleep to set the update frequency
			try {
				Thread.sleep(20);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
