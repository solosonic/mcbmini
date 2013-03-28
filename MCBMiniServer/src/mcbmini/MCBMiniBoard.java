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

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map.Entry;

import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.Command;
import mcbmini.MCBMiniConstants.Error;
import mcbmini.MCBMiniConstants.StreamMode;
import mcbmini.MCBMiniConstants.ExtraPinMode;
import mcbmini.MCBMiniConstants.FeedbackMode;
import mcbmini.MCBMiniConstants.MotorPolarity;
import mcbmini.MCBMiniConstants.ControlMode;
import mcbmini.functions.TargetFunction;

/**
 * @author siggi
 * @date Feb 17, 2011
 */
public class MCBMiniBoard {

	private int id;
	private boolean[] params_dirty;
	private boolean[] fresh_target;

	private TargetFunction[] target_functions;

	private EnumMap<ChannelParameter, Integer>[] params;
	private EnumMap<ChannelParameter, Integer>[] params_in_use;
	private EnumMap<Command, Integer> commands_in_use;

	private EnumMap<Error, Integer>[] error_counts;

	protected long last_received_message_ms;

	public MCBMiniBoard(int id){
		if( id < 0 || id > 126 ){
			throw new IllegalArgumentException("Id needs to be within 0 and 127");
		}

		this.id = id;
		params = new EnumMap[2];
		params_in_use = new EnumMap[2];
		commands_in_use = new EnumMap<Command, Integer>( Command.class );
		fresh_target = new boolean[2];
		error_counts = new EnumMap[2];

		target_functions = new TargetFunction[2];

		params_dirty = new boolean[2];

		/*
		 * Here we set the default parameters
		 */
		for (Channel channel : Channel.values()) {
			params[channel.index] = new EnumMap<ChannelParameter, Integer>(ChannelParameter.class);
			params_in_use[channel.index] = new EnumMap<ChannelParameter, Integer>(ChannelParameter.class);
			error_counts[channel.index] = new EnumMap<Error, Integer>(Error.class);
			fresh_target[channel.index] = true;
			params_dirty[channel.index] = true;

			for (ChannelParameter param : ChannelParameter.values()) {
				params[channel.index].put(param, 0);
			}

			// Making sure that some parameters don't get synchronized unless an explicit value gets set externally
			params_in_use[channel.index].put(ChannelParameter.PID_UPDATE_PERIOD, 0);

			setPositionPGain(channel, 50);
			setPositionDGain(channel, 20);
			setPositionIGain(channel, 5);
			setPositionDownscale(channel, 2);
			setVelocityPGain(channel, 50);
			setVelocityDGain(channel, 20);
			setVelocityIGain(channel, 5);
			setVelocityDownscale(channel, 2);
			setVelocityTimeDelta(channel, 2);

			setMaxVelocity(channel, 100);
			setMaxAcceleration(channel, 100);
			setPolarity(channel, MotorPolarity.REGULAR);
			setFeedbackMode(channel, FeedbackMode.ENCODER);
			setControlMode(channel, ControlMode.POSITION);
			setStreamMode(channel, StreamMode.OFF);
			setSlowEnableConstant(channel, 30);
		}
	}


	/**
	 * Sets any parameter of the board (including target positions)
	 * @param channel
	 * @param param
	 * @param value
	 */
	protected synchronized void setChannelParameter(Channel channel, ChannelParameter param, int value){
		params[channel.index].put(param, value);

		if( param.forward_to_board_on_change ) params_dirty[channel.index] = true;
	}
	protected synchronized void setChannelAParameter(ChannelParameter param, int value){ setChannelParameter(Channel.A, param, value); }
	protected synchronized void setChannelBParameter(ChannelParameter param, int value){ setChannelParameter(Channel.B, param, value); }

	protected synchronized int getChannelParameter(Channel channel, ChannelParameter param){
		return params[channel.index].get(param);
	}
	protected synchronized int getChannelAParameter(ChannelParameter param){ return getChannelParameter(Channel.A, param); }
	protected synchronized int getChannelBParameter(ChannelParameter param){ return getChannelParameter(Channel.B, param); }

	public void setPIDUpdatePeriod(int pid_update_period){
		limitCheck(pid_update_period, 0, 255);
		setChannelAParameter(ChannelParameter.PID_UPDATE_PERIOD, pid_update_period);
	}

	/*
	 * Target function related stuff
	 */
	public void setTargetFunction(Channel channel, TargetFunction function){
		if( function != null ){
			function.setOwner(this, channel);
			function.initialize();
		}
		this.target_functions[ channel.index ] = function;
	}

	public TargetFunction getTargetFunction(Channel channel){
		return this.target_functions[ channel.index ];
	}

	public void applyTargetFunction(Channel channel){
		TargetFunction targetFunction = target_functions[channel.index];
		if( targetFunction != null ){
			params[channel.index].put(ChannelParameter.TARGET_TICK, targetFunction.applyFunction());
			fresh_target[channel.index] = true;
		}
	}

	/*
	 * These are convenience methods to get/put parameters
	 */
	public int getPositionPGain(Channel channel){ return getChannelParameter(channel, ChannelParameter.POS_P_GAIN); }
	public void setPositionPGain(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.POS_P_GAIN, value); }

	public int getPositionIGain(Channel channel){ return getChannelParameter(channel, ChannelParameter.POS_I_GAIN); }
	public void setPositionIGain(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.POS_I_GAIN, value); }

	public int getPositionDGain(Channel channel){ return getChannelParameter(channel, ChannelParameter.POS_D_GAIN); }
	public void setPositionDGain(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.POS_D_GAIN, value); }

	public int getPositionDownscale(Channel channel){ return getChannelParameter(channel, ChannelParameter.POS_DOWNSCALE); }
	public void setPositionDownscale(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.POS_DOWNSCALE, value); }

	public int getVelocityPGain(Channel channel){ return getChannelParameter(channel, ChannelParameter.VEL_P_GAIN); }
	public void setVelocityPGain(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.VEL_P_GAIN, value); }

	public int getVelocityIGain(Channel channel){ return getChannelParameter(channel, ChannelParameter.VEL_I_GAIN); }
	public void setVelocityIGain(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.VEL_I_GAIN, value); }

	public int getVelocityDGain(Channel channel){ return getChannelParameter(channel, ChannelParameter.VEL_D_GAIN); }
	public void setVelocityDGain(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.VEL_D_GAIN, value); }

	public int getVelocityDownscale(Channel channel){ return getChannelParameter(channel, ChannelParameter.VEL_DOWNSCALE); }
	public void setVelocityDownscale(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.VEL_DOWNSCALE, value); }

	public int getVelocityTimeDelta(Channel channel){ return getChannelParameter(channel, ChannelParameter.VEL_TIME_DELTA); }
	public void setVelocityTimeDelta(Channel channel, int value){ limitCheckLow(value, 2); setChannelParameter(channel, ChannelParameter.VEL_TIME_DELTA, value); }

	public int getMaxVelocity(Channel channel){ return getChannelParameter(channel, ChannelParameter.MAX_VELOCITY); }
	public void setMaxVelocity(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.MAX_VELOCITY, value); }

	public int getMaxAcceleration(Channel channel){ return getChannelParameter(channel, ChannelParameter.MAX_ACCELERATION); }
	public void setMaxAcceleration(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.MAX_ACCELERATION, value); }

	public int getTargetTick(Channel channel){ return getChannelParameter(channel, ChannelParameter.TARGET_TICK); }

	public synchronized void setTargetTick(Channel channel, int value){
		target_functions[channel.index] = null;	// If someone sets the target position manually, then we disable any function that might be in control
		params[channel.index].put(ChannelParameter.TARGET_TICK, value);	// This is a bit faster
		fresh_target[channel.index] = true;
	}

	public synchronized Integer getFreshTargetTick(Channel channel){
		if( fresh_target[channel.index] ){
			fresh_target[channel.index] = false;
			return params[channel.index].get(ChannelParameter.TARGET_TICK);	// This is a bit faster
		}
		return null;
	}

	public synchronized int getActualTick(Channel channel){
		return params[channel.index].get(ChannelParameter.ACTUAL_TICK); // This is a bit faster
	}
	public int getMotorCurrent(Channel channel){ return getChannelParameter(channel, ChannelParameter.MOTOR_CURRENT); }

	public MotorPolarity getPolarity(Channel channel){ return MotorPolarity.getMotorPolarity(this, channel); }
	public void setPolarity(Channel channel, MotorPolarity value){ setChannelParameter(channel, ChannelParameter.POLARITY, value.id); }

	public FeedbackMode getFeedbackMode(Channel channel){ return FeedbackMode.getFeedbackMode(this, channel); }
	public void setFeedbackMode(Channel channel, FeedbackMode value){ setChannelParameter(channel, ChannelParameter.FEEDBACK_MODE, value.id); }

	public ExtraPinMode getExtraPinMode(Channel channel){ return ExtraPinMode.getExtraPinMode(this, channel); }
	public void setExtraPinMode(Channel channel, ExtraPinMode value){ setChannelParameter(channel, ChannelParameter.EXTRA_PIN_MODE, value.id); }

	public StreamMode getStreamMode(Channel channel){ return StreamMode.getStreamMode(this, channel); }
	public void setStreamMode(Channel channel, StreamMode value){ setChannelParameter(channel, ChannelParameter.STREAM_MODE, value.id); }

	public ControlMode getControlMode(Channel channel){ return ControlMode.getControlMode(this, channel); }
	public void setControlMode(Channel channel, ControlMode value){ setChannelParameter(channel, ChannelParameter.CONTROL_MODE, value.id); }

	public int getExtraPinValue(Channel channel){ return getChannelParameter(channel, ChannelParameter.EXTRA_PIN_VALUE); }
	public void setExtraPinValue(Channel channel, int value){ limitCheck(value, 0, 400); setChannelParameter(channel, ChannelParameter.EXTRA_PIN_VALUE, value); }

	public int getSlowEnableConstant(Channel channel){ return getChannelParameter(channel, ChannelParameter.SLOW_ENABLE_CONST); }
	public void setSlowEnableConstant(Channel channel, int value){ limitCheckLow(value, 0); setChannelParameter(channel, ChannelParameter.SLOW_ENABLE_CONST, value); }

	public boolean getEnabled(Channel channel){ return getChannelParameter(channel, ChannelParameter.ENABLED)==1; }
	public void setEnabled(Channel channel, boolean value){ setChannelParameter(channel, ChannelParameter.ENABLED, value?1:0); }

	public int getActualPotentiometerValue(Channel channel){ return getChannelParameter(channel, ChannelParameter.ACTUAL_POT); }
	public int getActualEncoderValue(Channel channel){ return getChannelParameter(channel, ChannelParameter.ACTUAL_ENCODER); }

	/**
	 * This method returns the set of parameters that need to be updated to a board
	 * @param channel
	 * @return null if no parameters need to be updated
	 */
	public synchronized HashMap<Command, Integer> getDirtyParameters(Channel channel){
		if( !params_dirty[channel.index] ) return null;

		HashMap<Command, Integer> out = new HashMap<Command, Integer>();

		/*
		 * First we see which ones of the actual ChannelParams need to be updated
		 */
		for (Entry<ChannelParameter, Integer> entry : params[channel.index].entrySet()) {
			if( !entry.getKey().forward_to_board_on_change ) continue;

			Integer val_in_use = params_in_use[channel.index].get( entry.getKey() );
			if( val_in_use == null ) val_in_use = -1;

			// If the value in use is not the same as the value specified by user, then it needs to be updated
			if( !val_in_use.equals( entry.getValue() ) ){
				out.put(entry.getKey().command, entry.getValue());
				params_in_use[channel.index].put(entry.getKey(), entry.getValue());
			}
		}

		// Reset the dirty flag
		params_dirty[channel.index] = false;
		return out;
	}

	/**
	 * this method should be called when the boards get reset
	 * @param channel
	 */
	protected void clearParametersInUse(Channel channel){
		params_in_use[channel.index].clear();
		commands_in_use.clear();

		params_dirty[channel.index] = true;
	}

	public synchronized int getId() {
		return id;
	}

	/**
	 * Returns the error count for a channel-specific error
	 * @param error
	 * @param channel
	 * @return
	 */
	public synchronized int getErrorCount(Error error, Channel channel){
		if( channel == null ){
			if( error.channel_specific ) throw new RuntimeException("Can't get non-channel specific error count for Error: "+error);
			channel = Channel.A;
		}
		Integer val = error_counts[channel.index].get(error);
		return val==null?0:val;
	}

	/**
	 * Returns the error count for a non-channel-specific error
	 * @param error
	 * @return
	 */
	public synchronized int getErrorCount(Error error){
		return getErrorCount(error, null);
	}

	/**
	 * Increases the error count for a channel-specific error
	 * @param error
	 * @param channel
	 * @return
	 */
	public synchronized void increaseErrorCount(Error error, Channel channel){
		if( channel == null ){
			if( error.channel_specific ) throw new RuntimeException("Can't increase non-channel specific error count for Error: "+error);
			channel = Channel.A;
		}
		Integer val = error_counts[channel.index].get(error);
		if( val == null ){
			error_counts[channel.index].put(error, 1);
		}
		else{
			error_counts[channel.index].put(error, (val+1));
		}
	}

	/**
	 * Increases the error count for a non-channel-specific error
	 * @param error
	 * @return
	 */
	public synchronized void increaseErrorCount(Error error){
		increaseErrorCount(error, null);
	}

	//@Override
	public String toString() {
		StringBuilder str = new StringBuilder("Board id: ");
		str.append(getId());
		str.append("; Errors: ");
		for (Error error : Error.values()) {
			str.append(error);
			str.append(":");
			if(!error.channel_specific){
				str.append(getErrorCount(error));
			}
			str.append("; ");
		}
		for (Channel ch : Channel.values()) {
			str.append("\nChannel ");
			str.append(ch);
			str.append(": ");
			for (ChannelParameter param : ChannelParameter.values()) {
				str.append(param);
				str.append(":");
				str.append(getChannelParameter(ch, param));
				str.append("; ");
			}
			for (Error error : Error.values()) {
				str.append(error);
				str.append(":");
				if(error.channel_specific){
					str.append(getErrorCount(error, ch));
				}
				str.append("; ");
			}
			str.append("\n");
		}

		return str.toString();
	}

	private void limitCheck(int val, int low, int high){
		if( val < low || val > high ) throw new IllegalArgumentException("Value must be between (and including) "+low+" and "+high+", entered: "+val);
	}
	private void limitCheckLow(int val, int low){
		if( val < low ) throw new IllegalArgumentException("Error in: \""+new RuntimeException().getStackTrace()[1]+"\" value must be >= "+low+", entered: "+val);
	}
}
