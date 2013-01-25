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

package mcbmini.utils;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;

import javax.swing.Timer;

import mcbmini.serial.PSerial;

/**
 * @author siggi
 * @date Feb 4, 2011
 */
public class BluetoothSetup {

	private static PSerial ser;

	public static void main(String[] args) {
		/*
		 * This method lists the available ports and gives you names of the ports
		 * This call has been known to take some time (especially on Win) so if you
		 * know the name of your port then it could be good not to make it frequently at least
		 */

		String port = null;
		if( args.length == 0 ){
			String[] avalable_ports = PSerial.list();

			for (String string : avalable_ports) {
				System.out.println(string);
				if( string.contains("FireFly") ){
					port = string;
					break;
				}
			}
		}
		else{
			port = args[0];
		}

		if(port == null){
			throw new RuntimeException("Coulnd't find FireFly port");
		}

		/*
		 * Here we init the port, if the port is occupied or doesn't exist then this will throw
		 * an exception.
		 */

		System.out.println("Starting serial....");
		try {
			ser = new PSerial( port, 115200);
		} catch (IOException e) {

			e.printStackTrace();
			System.exit(0);
		}
		System.out.println("Done!");

		/*
		 * This is good coding practice but might be unneccesary as the serial library might
		 * have its own shutdownhooks
		 */
		Runtime.getRuntime().addShutdownHook( new Thread(new Runnable() {

//			//@Override
			public void run() {
				ser.stop();
			}
		}));

		System.out.println("This should happen in the within 60 seconds of you bluetooth device starting up");
		System.out.println("");

		Timer t = new Timer(20, new ActionListener() {

			long start = -1;
			int stage = 0;
			String reply = "";

//			//@Override
			public void actionPerformed(ActionEvent arg0) {
				if( start == -1 ) start = System.currentTimeMillis();
				long passed = System.currentTimeMillis() - start;

				int available = ser.available();
				if( available > 0 ){
					byte[] received = ser.readBytes();

					// Do something with the received buffer
					reply = new String(received);
					System.out.println("BlueSmirf replied: " + reply);
				}

				try {

					if( stage == 0 ){
						System.out.println("Start CMD mode...");
						byte[] bla = getBytesForString("$$$");
						ser.write( bla );
						reply = "";
						stage++;
					}

					else if(stage == 1 && reply.contains("CMD")){
						System.out.println("Set in 115.2K baud rate mode...");
						byte[] bla = getBytesForString("SU,11\r");
						ser.write( bla );
						reply = "";
						stage++;
					}

					else if(stage == 2 && reply.contains("AOK")){
						System.out.println("End CMD mode...");
						byte[] bla = getBytesForString("---\r");
						ser.write( bla );
						reply = "";
						stage++;
					}

					else if(stage == 3 && reply.contains("END")){
						System.out.println("Done...");
						System.exit(0);
					}
				}
				catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		});
		t.start();
	}

	private static byte[] getBytesForString(String in){
		char[] bla = in.toCharArray();
		byte[] ret = new byte[bla.length];
		for(int i=0; i<bla.length; i++){
			ret[i] = (byte)bla[i];
		}
		return ret;
	}
}
