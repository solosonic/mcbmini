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

package mcbmini.serial;

import java.io.IOException;

/**
 * User: jg
 * Date: 4/4/11
 * Time: 11:37 PM
 */
public interface iSerial {

	public void addSerialEventHandler(SerialEventHandler handler);

	public void write(byte[] bytes, int offset, int length) throws IOException;

	/**
	 * Returns the number of bytes that have been read from serial
	 * and are waiting to be dealt with by the user.
	 * @return number of bytes available
	 */
	public int available();

	/**
	 * Returns a number between 0 and 255 for the next byte that's
	 * waiting in the buffer.
	 * @return next byte, or -1 if there was no byte (although the user should first check available() to see if things are ready to avoid this)
	 */
	public byte readByte();

	/**
	 * Flag for indicating whether a serial interface is ready
	 * @return true when the serial interface is ready
	 */
	public boolean isInitialized();
}
