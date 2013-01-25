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

 /**
 * @author asetapen
 * @date Jul 18, 2011
 */

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import mcbmini.serial.iSerial;

public class AndroidIOIOMCBMiniServer extends MCBMiniServer {


	public AndroidIOIOMCBMiniServer(ArrayList<MCBMiniBoard> boards, int ioio_rx_pin, int ioio_tx_pin) throws IOException{
		super(createMCBMiniSerialManager(ioio_rx_pin, ioio_tx_pin), boards);
	}

	private static MCBMiniSerialManager createMCBMiniSerialManager(int ioio_rx_pin, int ioio_tx_pin){
		System.out.println("IOIO call: AndroidIOIOMCBMiniServer");
		iSerial ser = new AndroidIOIOPSerial(ioio_rx_pin, ioio_tx_pin);

		int cnt = 0;
		while (!ser.isInitialized()) {
			try {
				Thread.sleep(10);
				if (cnt++ > 500) {
					System.out.println("Timeout waiting for the IOIO");
					throw new RuntimeException("Timeout in waiting for the IOIO serial initialization");
				}
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		return new MCBMiniSerialManager(ser);
	}
}
