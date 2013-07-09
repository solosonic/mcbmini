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
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;

import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.Command;
import mcbmini.MCBMiniConstants.DataSize;
import mcbmini.MCBMiniConstants.Id;
import mcbmini.serial.PSerial;
import mcbmini.serial.iSerial;
import mcbmini.utils.ByteBufferUtils;
import mcbmini.utils.Log;


/**
 * @author siggi
 * @date Feb 20, 2011
 */
public class MCBMiniSerialManager {

	public static final boolean DEBUG = false;
	public static final boolean PRINT_RX_BYTES = false;

// This effectively guides the max framerate of the system by making sure that no packet is shorter than this many bytes
// This also makes sure that whenever a message is going out from the master, the slave that received the previous message has enough time to respond before the next one does
// If this is too low then we start getting packet collisions

	public static final int MIN_MASTER_PACKET_SIZE_OLD_FIRMWARE = 25;
	public static final int MIN_MASTER_PACKET_SIZE_NEW_FIRMWARE = 17;
	private int minMasterPacketSize = MIN_MASTER_PACKET_SIZE_OLD_FIRMWARE;
	
	public static final byte HEADER_BYTE = (byte)0xAA;
	public static final byte ESCAPE_BYTE = (byte)0x55;

	private long escape_bytes_received;
	private long total_bytes_received;
	
	private ByteBuffer read_bb;
//	private byte checksum = 0;

	private byte[] temp_buffer_bytes;
	private ByteBuffer temp_buffer;

	private byte[] write_buffer_bytes;
	public ByteBuffer write_buffer;

	private byte[] zero_bytes;

	protected iSerial ser;

	protected int bad_checksum_received_counter;

	public MCBMiniSerialManager(String port_name, int baud_rate) throws IOException{

		ser = new PSerial(port_name, baud_rate);
		init();
	}
	
	public MCBMiniSerialManager(iSerial pSerial){
		this.ser = pSerial;
		init();
	}

	protected void init(){
		read_bb = ByteBuffer.allocate(1024);

		temp_buffer_bytes = new byte[128];
		temp_buffer = ByteBuffer.wrap(temp_buffer_bytes);

		write_buffer_bytes = new byte[10000];
		write_buffer = ByteBuffer.wrap(write_buffer_bytes);

		zero_bytes = new byte[128];
		bad_checksum_received_counter = 0;
		
		escape_bytes_received = 0;
		total_bytes_received = 0;
	}

	protected void setMinMasterPacketSize(int newValue){
		minMasterPacketSize = newValue;
	}
	
	public int getNumberOfBadChecksums(){
		return bad_checksum_received_counter;
	}

	private boolean next_byte_should_be_transformed = false;

	public List<ByteBuffer> handleBufferedSerialData() {
		ArrayList<ByteBuffer> return_buffers = new ArrayList<ByteBuffer>(1);

		while( ser.available() > 0 ){
			byte in = ser.readByte();

			if( PRINT_RX_BYTES ){
				Log.println("Received byte "+(in&0xff), true);
			}

			// We might have a ready packet !
			if( in == HEADER_BYTE ){
				if( read_bb.position() < 3 ){
					Log.println("Packet stub received");
					read_bb.clear();
					continue;
				}

				// Extract checksum from packet
				byte checksum_rcv = read_bb.get( read_bb.position() - 1 );
				read_bb.position( read_bb.position()-1 );

				// Now we calculate the checksum in the package using the expected number of bytes
				byte cmd = read_bb.get( read_bb.position()-2 );
				Command command = MCBMiniConstants.Command.getForCmdId(cmd & 0xff);
				int expectedNumberOfBytes = 2 + command.datasize.number_of_bytes;
				
				if( read_bb.position() < expectedNumberOfBytes ){
					Log.println("Improper packet size");
					ByteBufferUtils.printByteBuffer(read_bb, read_bb.position());
					read_bb.clear();
					continue;
				}

				// Make a new packet
				read_bb.limit( read_bb.position() );
				read_bb.position( read_bb.position()-expectedNumberOfBytes );
				ByteBuffer packet = ByteBuffer.allocate(64);
				packet.order(ByteOrder.LITTLE_ENDIAN);
				packet.put( read_bb );
				packet.flip();
				
				// Calculate checksum
				byte checksum_calculated = 0;
				for(int i=0; i<packet.limit(); i++){
					checksum_calculated += packet.get(i);
				}
				
				// If the checksums don't match up
				if( checksum_calculated != checksum_rcv ){
					Log.println("Packet with bad checksum received ! calculated: "+ByteBufferUtils.byte2int(checksum_calculated)+", received: "+ByteBufferUtils.byte2int(checksum_rcv));
					ByteBufferUtils.printByteBuffer(packet, packet.limit());
					bad_checksum_received_counter++;
					read_bb.clear();
					continue;
				}

				// Add the packet buffer to be handled
				return_buffers.add( packet );
				read_bb.clear();
				continue;
			}

			if( in == ESCAPE_BYTE ){
				escape_bytes_received++;
				next_byte_should_be_transformed = true;
				continue;
			}

			if( next_byte_should_be_transformed ){
				in ^= 1;
				next_byte_should_be_transformed = false;
			}

			total_bytes_received++;
			read_bb.put(in);
		}
		return return_buffers;
	}

	/**
	 * Sending functions
	 */

	public void sendTxBuffer(){
		try {
			ser.write(write_buffer_bytes, 0, write_buffer.position());
		} catch (IOException e) {
			e.printStackTrace();
			System.exit(1);
		}

		write_buffer.clear();
	}

	private byte write_checksum = 0;
	private void startSubCommand(){
		temp_buffer.clear();
		write_checksum = 0;
	}

	private void finishSubCommandAndAddToBuffer(){

		putByteInSubCommand(write_checksum);
		temp_buffer.put(HEADER_BYTE);

		if( DEBUG ){
			String str = "Writing out bytes: ";
			for (int i = 0; i < temp_buffer.position(); i++) {
				str += ByteBufferUtils.byte2int( temp_buffer.get(i) ) + ", ";
			}
			Log.println(str, true);
		}

		// Pad with zeros because of slave bus contention issues
		// This ensures that no master write packet is smaller than MIN_MASTER_PACKET_SIZE
		if( temp_buffer.position() < minMasterPacketSize ){
			write_buffer.put(zero_bytes, 0, minMasterPacketSize-temp_buffer.position());
		}

		// Put the subcommand into the buffer
		temp_buffer.flip();
		write_buffer.put( temp_buffer );
	}

	private void putByteInSubCommand(byte in){
		write_checksum += in;
		if( in == HEADER_BYTE || in == ESCAPE_BYTE ){
			in ^= 1;
			temp_buffer.put(ESCAPE_BYTE);
		}
		temp_buffer.put(in);
	}

	private void putIntInSubCommand(int in){
		// Reversed because the buffer is reversed
		putByteInSubCommand( (byte)((in)&0xff) );
		putByteInSubCommand( (byte)((in>>8)&0xff) );
		putByteInSubCommand( (byte)((in>>16)&0xff) );
		putByteInSubCommand( (byte)((in>>24)&0xff) );
	}

	public static enum ResponseType{
		MOTOR_CURRENT 		(Command.TWO_TARGET_TICK_MOTOR_CURRENT),
		MOTOR_CURRENT_TWO 	(Command.TWO_TARGET_TICK_TWO_MOTOR_CURRENT),
		ACTUAL_TICK			(Command.TWO_TARGET_TICK_ACTUAL),
		ACTUAL_TICK_TWO		(Command.TWO_TARGET_TICK_TWO_ACTUAL),
		ACTUAL_VELOCITY 	(Command.TWO_TARGET_TICK_VELOCITY),
		ACTUAL_VELOCITY_TWO	(Command.TWO_TARGET_TICK_TWO_VELOCITY),
		ACTUAL_POT			(Command.TWO_TARGET_TICK_POT),
		ACTUAL_POT_TWO		(Command.TWO_TARGET_TICK_TWO_POT),
		ACTUAL_ENCODER		(Command.TWO_TARGET_TICK_ENCODER),
		ACTUAL_ENCODER_TWO	(Command.TWO_TARGET_TICK_TWO_ENCODER);

		public Command feedbackCommand;
		private ResponseType(Command feedbackCommand){
			this.feedbackCommand = feedbackCommand;
		}
	}

	/**
	 * This command should only be used for the two-target special comm
	 * @param board
	 * @param channel
	 * @param command
	 * @param request_response
	 * @param targetA
	 * @param targetB
	 */
	protected void writeSpecializedPacket(MCBMiniBoard board, ResponseType feedback_type, Channel feedback_channel, int targetA, int targetB){
		startSubCommand();

		byte command_byte = feedback_type.feedbackCommand.command;

		putIntInSubCommand( targetB );
		putIntInSubCommand( targetA );

		byte id_byte = (byte)(board.getId());

		if( feedback_channel == Channel.B ){
			id_byte |= 0x80;
		}

		putByteInSubCommand(command_byte);
		putByteInSubCommand(id_byte);

		finishSubCommandAndAddToBuffer();
	}

	/**
	 * This method should be used to write any generic packet to the boards (except the two-target special commands, use writeSpecializedPacket for those)
	 * @param board
	 * @param channel
	 * @param command
	 * @param request_response
	 * @param value
	 */
	protected void writeGenericPacket(MCBMiniBoard board, Channel channel, Command command, boolean request_response, int value){
		startSubCommand();

		if(DEBUG){
			if( board != null )
				Log.println("Sending command "+ByteBufferUtils.byte2int(command.command)+" to board "+board.getId()+" channel "+channel, true);
			else
				Log.println("Sending command "+ByteBufferUtils.byte2int(command.command)+" to all boards  channel "+channel, true);
		}


		byte command_byte = command.command;
		if( request_response ){
			command_byte |= 0x80;
		}
		// Otherwise if we are sending a non request for response packet
		else{
			if( command.datasize == null ){
				throw new RuntimeException("Cannot send this command to boards with a value "+command);
			}

			// This is an extra paranoid crc like check for writing IDs to the boards
			if( command == Command.ID ){
				putByteInSubCommand( (byte)(value+10) );
				putByteInSubCommand( (byte)value );
				putByteInSubCommand( (byte)3 );
				putByteInSubCommand( (byte)2 );
				putByteInSubCommand( (byte)1 );
			}
			// Otherwise it is just a regular packet
			else{
				if( command.datasize == DataSize.S32 ){
					putIntInSubCommand( value );
				}
				else if( command.datasize == DataSize.U08 ){
					putByteInSubCommand( (byte)value );
				}
			}
		}

		byte id_byte;
		if( board==null ) id_byte = Id.BROADCAST.getID();
		else id_byte = (byte)(board.getId());

		if( channel == Channel.B ){
			id_byte |= 0x80;
		}

		putByteInSubCommand(command_byte);
		putByteInSubCommand(id_byte);

		finishSubCommandAndAddToBuffer();
	}

	/**
	 * Writes all the parameter values of the board to the actual board (minus the ones specified in the "params_to_skip_in_param_packet"
	 * @param board
	 * @param channel
	 */
	public void writeParameterPacket(MCBMiniBoard board, Channel channel, HashMap<Command, Integer> parameters){
		for (Entry<Command, Integer> param_entry : parameters.entrySet()) {
			if( !param_entry.getKey().equals( Command.ENABLE ) ){
				writeGenericPacket(board, channel, param_entry.getKey(), false, param_entry.getValue());
			}
		}
		// We always have to send the enabled value (true or false) at the end of a parameter sequence to take the board out of initialization mode
		writeGenericPacket(board, channel, Command.ENABLE, false, board.getChannelParameter(channel, ChannelParameter.ENABLED));
	}
}
