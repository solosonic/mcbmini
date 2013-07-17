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

import java.util.HashMap;

/**
 * @author siggi
 * @date Feb 17, 2011
 */
public interface MCBMiniConstants {

	public static enum Id{
		BROADCAST		((byte)127),
		INVALID			((byte)126);

		public byte getID(){
			return id;
		}

		public int getIDInt(){
			return (int)(id & 0xff);
		}

		private byte id;
		private Id(byte id) {
			this.id = id;
		}
	}

	public static enum ChannelParameter{
		POS_P_GAIN				( Command.POS_P_GAIN, true ),
		POS_D_GAIN				( Command.POS_D_GAIN, true ),
		POS_I_GAIN				( Command.POS_I_GAIN, true ),
		POS_DOWNSCALE			( Command.POS_DOWNSCALE, true ),

		VEL_P_GAIN				( Command.VEL_P_GAIN, true ),
		VEL_D_GAIN				( Command.VEL_D_GAIN, true ),
		VEL_I_GAIN				( Command.VEL_I_GAIN, true ),
		VEL_DOWNSCALE			( Command.VEL_DOWNSCALE, true ),
		VEL_TIME_DELTA			( Command.VEL_TIME_DELTA, true ),

		MAX_VELOCITY			( Command.MAX_VELOCITY, true ),
		MAX_ACCELERATION		( Command.MAX_ACCELERATION, true ),

		TARGET_TICK				( Command.TARGET_TICK, false ),
		ACTUAL_TICK				( Command.ACTUAL_TICK, false ),
		MOTOR_CURRENT			( Command.MOTOR_CURRENT, false ),
		POLARITY				( Command.POLARITY, true),
		ENABLED					( Command.ENABLE, true ),
		FEEDBACK_MODE			( Command.FEEDBACK_MODE, true ),
		CONTROL_MODE			( Command.CONTROL_MODE, true ),
		SLOW_ENABLE_CONST		( Command.SLOW_ENABLE_CONST, true ),
		EXTRA_PIN_MODE			( Command.EXTRA_PIN_MODE, true ),
		EXTRA_PIN_VALUE			( Command.EXTRA_PIN_VALUE, false ),

		STREAM_MODE				( Command.STREAM_MODE, true ),

		ACTUAL_POT				( Command.POT_VALUE, false ),
		ACTUAL_ENCODER			( Command.ENCODER_VALUE, false ),

		PID_UPDATE_PERIOD		( Command.PID_UPDATE_PERIOD, true ),

		TARGET_MIN				( null, false ),
		TARGET_MAX				( null, false ),
		TARGET_DEFAULT			( null, false );

		public Command command;
		public boolean forward_to_board_on_change;
		private ChannelParameter(Command command, boolean forward_to_board_on_change) {
			this.command = command;
			this.forward_to_board_on_change = forward_to_board_on_change;
		}
	}

	public enum Channel {
		A(0),
		B(1);

		public int index;
		Channel(int index){ this.index = index; }

		public static Channel getForId(int id){
			if( id==A.index ) return A;
			else return B;
		}
	};

	public enum DataSize {
		ZERO		(0),
		U08			(1),
		S32			(4),
		DOUBLE_S32	(8);

		public int number_of_bytes;

		DataSize(int number_of_bytes){
			this.number_of_bytes = number_of_bytes;
		}

	};

	public enum Command {
		ID								( 0 , DataSize.U08 ),

		POS_P_GAIN						( 1 , DataSize.S32 ),
		POS_I_GAIN						( 2 , DataSize.S32 ),
		POS_D_GAIN						( 3 , DataSize.S32 ),
		MAX_VELOCITY					( 4 , DataSize.S32 ),

		ENABLE							( 6 , DataSize.U08 ),
		POLARITY						( 7 , DataSize.U08 ),
		FEEDBACK_MODE					( 8 , DataSize.U08 ),
		CONTROL_MODE					( 9 , DataSize.U08 ),

		TARGET_TICK						( 10 , DataSize.S32 ),	// This command sets/requests target positions
		ACTUAL_TICK						( 11 , DataSize.S32 ),	// This command sets/requests actual positions
		MOTOR_CURRENT					( 12 , DataSize.S32 ),	// This command sets/requests motor currents

		TWO_TARGET_TICK_MOTOR_CURRENT	( 13 , DataSize.S32 ),	// This command sets the target positions for both channels and requests motor current back
		TWO_TARGET_TICK_ACTUAL			( 15 , DataSize.S32 ),	// This command sets the target positions for both channels and requests actual position back

		EMPTY_RESPONSE					( 17 , DataSize.ZERO ),
		ERROR							( 18 , DataSize.U08 ),
		PID_OUTPUT						( 19 , DataSize.S32 ),

		POS_DOWNSCALE					( 21 , DataSize.U08 ),	// Helps provide more resolution in the PID parameters

		// These following only work for firmware versions >= 1
		ENCODER_VALUE					( 22 , DataSize.S32 ),		// Sets/gets the current encoder value
		POT_VALUE						( 23 , DataSize.S32 ),		// Gets the current pot value
		FIRMWARE_VERSION				( 24 , DataSize.S32 ),		// Gets the version of the firmware

		// These following only work for firmware versions >= 2
		MAX_PWM_DUTY_CYCLE				( 25 , DataSize.S32 ),		// Sets the maximum pwm duty cycle (can be used to make sure that the output is "appropriate" for 6V motors with a 12V supply. The full (and default) value is 1100

		// These following only work for firmware versions >= 6
		SLOW_ENABLE_CONST				( 26 , DataSize.U08 ),		// Sets the slow enable time constant approximately by this formula: rising_time [sec] = SLOW_ENABLE_CONST * 7 / 200
		DEBUG							( 27 , DataSize.S32 ),		// Not needed after the implementation of the message buffer

		// These following only work for firmware versions >= 9
		ACTUAL_ENCODER_OFFSET			( 28 , DataSize.S32 ),		// Allows host to offset the current ticks on the board by a number (should be used on reset of the board for example)

		// These following only work for firmware versions >= 11
		SATURATION						( 29 , DataSize.U08 ),		// Tells us whether the PID output is in negative saturation (1), positive saturation (2) or not saturated (0)
		I_COMPONENT						( 30 , DataSize.S32 ),		// Is the integrated I component of the PID

		// These following only work for firmware versions >= 14
		REQUEST_MESSAGE					( 31 , DataSize.ZERO ),		// This request is made by the master once it receives notice that a slave has messages
		EXTRA_PIN_MODE					( 32 , DataSize.U08 ),		// This sets the mode of the EXTRA pins according to ExtraMode
		EXTRA_PIN_VALUE					( 33 , DataSize.S32 ),		// This either sets the servo value (if the pin is in servo mode) or requests analog or switch sensor value

		TWO_TARGET_TICK_VELOCITY		( 34 , DataSize.S32 ),		// This command sets the target positions for both channels and requests velocity back

		// version >= 16
		ACTUAL_VEL						( 35 , DataSize.S32 ),		// Requests the actual velocity (tick difference)
		VEL_P_GAIN						( 36 , DataSize.S32 ),
		VEL_I_GAIN						( 37 , DataSize.S32 ),
		VEL_D_GAIN						( 38 , DataSize.S32 ),
		VEL_DOWNSCALE					( 39 , DataSize.U08 ),		// Helps provide more resolution in the PID parameters
		MAX_ACCELERATION				( 40 , DataSize.S32 ),		// Sets the maximum increase in velocity between updates
		VEL_TIME_DELTA					( 41 , DataSize.U08 ),		// Sets across how many updates the velocity (tick difference) should be calculated
		STREAM_MODE						( 42 , DataSize.U08 ),		// Sets whether we are streaming positions or not

		 // For V4.3 this is >= 17
		TWO_TARGET_TICK_POT				( 43 , DataSize.S32 ),		// This command sets the target positions for both channels and requests specifically the current pot value back
		TWO_TARGET_TICK_ENCODER			( 44 , DataSize.S32 ),		// This command sets the target positions for both channels and requests specifically the current encoder value back

		TWO_TARGET_TICK_TWO_ACTUAL			( 45 , DataSize.DOUBLE_S32 ),		// This command sets the target positions for both channels and requests the current actual value back for both channels
		TWO_TARGET_TICK_TWO_VELOCITY		( 46 , DataSize.DOUBLE_S32 ),		// This command sets the target positions for both channels and requests the current velocity back for both channels
		TWO_TARGET_TICK_TWO_MOTOR_CURRENT	( 47 , DataSize.DOUBLE_S32 ),		// This command sets the target positions for both channels and requests the electrical motor current value back for both channels
		TWO_TARGET_TICK_TWO_POT				( 48 , DataSize.DOUBLE_S32 ),		// This command sets the target positions for both channels and requests the potentiometer value back for both channels
		TWO_TARGET_TICK_TWO_ENCODER			( 49 , DataSize.DOUBLE_S32 ),		// This command sets the target positions for both channels and requests the encoder value back for both channels

		// This is for firmware versions > 17 for V4.4 and > 30 for V4.3
		PID_UPDATE_PERIOD					( 50 , DataSize.U08 );		// This value sets the update rate of the PID (larger number, slower update rate) 195 is the default and is approximately 100Hz

		public final byte command;
		public final DataSize datasize;

		Command(int cmd, DataSize size) {
			this.command = (byte)cmd;
			this.datasize = size;
		}

		private static Command[] id_to_command;
		public static Command getForCmdId(int id){
			if( id_to_command==null ){
				id_to_command = new Command[256];
				for (Command cmd : Command.values()) {
					id_to_command[cmd.command & 0xff] = cmd;
				}
			}
			return id_to_command[id];
		}
	}

	public enum Error{
		BAD_CHECKSUM					( 0, false ),
		BAD_CMD_RCV						( 1, false ),
		UNINITIALIZED					( 2, true ),
		BUF_OVF							( 3, false ),
		TIMEOUT_DISABLE					( 4, false ),
		FAULT							( 5, true ),
		BAD_ID_RCV						( 6, false ),
		PACKAGE_OVF						( 7, false ),
		PARAM_DUR_EN					( 9, true ),
		MSG_BUF_OVF						( 10, false ),
		NO_RESPONSE						( 255, false );	// This is not an error that the boards report but the host detects it

		public final byte id;
		public final boolean channel_specific;

		Error(int id, boolean channel_specific){
			this.id = (byte)id;
			this.channel_specific = channel_specific;
		}

		private static HashMap<Integer, Error> id_to_command;
		public static Error getForId(int id){
			if( id_to_command==null ){
				id_to_command = new HashMap<Integer, MCBMiniConstants.Error>();
				for (Error cmd : Error.values()) {
					id_to_command.put(Integer.valueOf( cmd.id&0xff ), cmd);
				}
			}
			return id_to_command.get( Integer.valueOf( id ));
		}
	}

	public enum ExtraPinMode{
		OFF			( 0 ),
		SWITCH		( 1 ),
		ANALOG		( 2 ),
		SERVO		( 3 );

		public final int id;
		ExtraPinMode(int id){
			this.id = id;
		}

		private static HashMap<Integer, ExtraPinMode> id_to_command;
		public static ExtraPinMode getForId(int id){
			if( id_to_command==null ){
				id_to_command = new HashMap<Integer, ExtraPinMode>();
				for (ExtraPinMode cmd : ExtraPinMode.values()) {
					id_to_command.put(Integer.valueOf( cmd.id&0xff ), cmd);
				}
			}
			return id_to_command.get( Integer.valueOf( id ));
		}

		public static ExtraPinMode getExtraPinMode(MCBMiniBoard board, Channel channel){
			return getForId( board.getChannelParameter(channel, ChannelParameter.EXTRA_PIN_MODE) );
		}
	}


	public enum FeedbackMode{
		ENCODER					( 0 ),
		POT						( 1 );

		public final int id;
		FeedbackMode(int id){
			this.id = id;
		}

		public static FeedbackMode getForId(int id){
			if( id == 0 ) return ENCODER;
			else return POT;
		}

		public static FeedbackMode getFeedbackMode(MCBMiniBoard board, Channel channel){
			return getForId( board.getChannelParameter(channel, ChannelParameter.FEEDBACK_MODE) );
		}
	}

	public enum MotorPolarity{
		REGULAR						( 0 ),
		FLIPPED						( 1 );

		public final int id;
		MotorPolarity(int id){
			this.id = id;
		}

		public static MotorPolarity getForId(int id){
			if( id == 0 ) return REGULAR;
			else return FLIPPED;
		}

		public static MotorPolarity getMotorPolarity(MCBMiniBoard board, Channel channel){
			return getForId( board.getChannelParameter(channel, ChannelParameter.POLARITY) );
		}
	}

	public enum ControlMode{
		POSITION						( 0 ),
		VELOCITY						( 1 ),
		MIXED							( 2 );

		public final int id;
		ControlMode(int id){
			this.id = id;
		}

		private static HashMap<Integer, ControlMode> map;
		public static ControlMode getForId(int id){
			if( map==null ){
				map = new HashMap<Integer, ControlMode>();
				for (ControlMode cmd : ControlMode.values()) {
					map.put(Integer.valueOf( cmd.id ), cmd);
				}
			}
			return map.get(id);
		}

		public static ControlMode getControlMode(MCBMiniBoard board, Channel channel){
			return getForId( board.getChannelParameter(channel, ChannelParameter.CONTROL_MODE) );
		}
	}

	public enum StreamMode{
		OFF			( 0 ),
		ON			( 1 );

		public final int id;
		StreamMode(int id){
			this.id = id;
		}

		private static HashMap<Integer, StreamMode> map;
		public static StreamMode getForId(int id){
			if( map==null ){
				map = new HashMap<Integer, StreamMode>();
				for (StreamMode cmd : StreamMode.values()) {
					map.put(Integer.valueOf( cmd.id ), cmd);
				}
			}
			return map.get(id);
		}

		public static StreamMode getStreamMode(MCBMiniBoard board, Channel channel){
			return getForId( board.getChannelParameter(channel, ChannelParameter.CONTROL_MODE) );
		}
	}
}
