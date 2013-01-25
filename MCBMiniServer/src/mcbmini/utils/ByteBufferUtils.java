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


import java.nio.ByteBuffer;

public class ByteBufferUtils {

	public static byte getFromBack(ByteBuffer bb){
		byte ret = bb.get(bb.limit()-1);
		bb.limit(bb.limit()-1);
		return ret;
	}

	public static int getIntFromBack(ByteBuffer bb){
		byte hh = bb.get(bb.limit()-4);
		byte hl = bb.get(bb.limit()-3);
		byte lh = bb.get(bb.limit()-2);
		byte ll = bb.get(bb.limit()-1);
		bb.limit(bb.limit()-4);

		return assembleS32(ll, lh, hl, hh);
	}

	public static int getIntFromBackReversed(ByteBuffer bb){
		byte hh = bb.get(bb.limit()-1);
		byte hl = bb.get(bb.limit()-2);
		byte lh = bb.get(bb.limit()-3);
		byte ll = bb.get(bb.limit()-4);
		bb.limit(bb.limit()-4);

		return assembleS32(ll, lh, hl, hh);
	}


	public static void writeUnsignedShort(int value, ByteBuffer bb){
		if( value < 0 || value >= 65536  ){
			System.err.println("Value can not be converted into unsigned short");
			return;
		}
		bb.put( (byte)((value>>8)&0xff) );
		bb.put( (byte)((value)&0xff) );
	}

	public static int readUnsignedShort(ByteBuffer bb){
		int h = (int)bb.get()&0xff;
		int l = (int)bb.get()&0xff;
		return h*256+l;
	}

	public static int readByteAsInt(ByteBuffer bb){
		byte b = bb.get();
		return (int)(b&0xff);
	}

//
//	public static int readS16(ByteBuffer bb){
//		byte h = bb.get();
//		byte l = bb.get();
//
//		return
//	}

	public static int assembleS32(byte ll, byte lh, byte hl, byte hh){
		int ret = ((hh & 0x000000FF) << 24) + ((hl & 0x000000FF) << 16) + ((lh & 0x000000FF) << 8) + (ll & 0x000000FF);
		return ret;
	}


	public static String byteBufferToString(ByteBuffer bb){
		String ret = "";
		for(int i=0; i<bb.position(); i++)
			ret += ((int)bb.get(i)&0xff) + " ";

		return ret;
	}

	public static int byte2int(byte in){
		return (int)in&0xff;
	}

	public static byte[] int2bytes(int in){
		byte[] bytes = new byte[4];
		bytes[0] = (byte)(in&0xff);
		bytes[1] = (byte)((in>>8)&0xff);
		bytes[2] = (byte)((in>>16)&0xff);
		bytes[3] = (byte)((in>>24)&0xff);
		return bytes;
	}

	public static void printByteBuffer(ByteBuffer bb, int how_many){
		for(int i=0; i<how_many; i++){
			System.out.print((int)(bb.get(i)&0xff) + " ");
		}
		System.out.println(" ");
	}

	public static byte calcChecksum(ByteBuffer bb){
		byte checksum_calc = 0;
		for(int i=0; i<bb.position(); i++)
			checksum_calc += bb.get(i);
		return checksum_calc;
	}

	public static void printByteArray(byte[] array, int cnt){
		for(int i=0; i<cnt; i++)
			System.out.print(unsignedByte2int(array[i])+" ");
		System.out.println(" ");

	}

	public static int unsignedByte2int(byte in){
		return (int)(in&0x000000ff);
	}

	public static int unsignedChar2int(byte in){
		return (int)(in&0x000000ff);
	}

}
