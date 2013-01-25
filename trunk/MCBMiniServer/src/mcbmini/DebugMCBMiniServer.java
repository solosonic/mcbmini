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

import java.io.IOException;
import java.util.ArrayList;

import mcbmini.serial.SerialEventHandler;
import mcbmini.serial.iSerial;

/**
 * @author siggi
 * @date Jul 16, 2012
 */
public class DebugMCBMiniServer extends MCBMiniServer{


	/**
	 * @param boards
	 * @throws IOException
	 */
	public DebugMCBMiniServer(ArrayList<MCBMiniBoard> boards) throws IOException {
		super(new MCBMiniSerialManager(new DebugSerial()), boards);
		// TODO Auto-generated constructor stub
	}

	private static class DebugSerial implements iSerial {

		//@Override
		public void addSerialEventHandler(SerialEventHandler handler) {
			// TODO Auto-generated method stub

		}

		//@Override
		public void write(byte[] bytes, int offset, int length) throws IOException {
			// TODO Auto-generated method stub

		}

		//@Override
		public int available() {
			// TODO Auto-generated method stub
			return 0;
		}

		//@Override
		public byte readByte() {
			// TODO Auto-generated method stub
			return 0;
		}

		//@Override
		public boolean isInitialized() {
			// TODO Auto-generated method stub
			return true;
		}

	}
}
