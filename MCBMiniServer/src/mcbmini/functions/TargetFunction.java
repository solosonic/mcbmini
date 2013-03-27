package mcbmini.functions;

import mcbmini.MCBMiniBoard;
import mcbmini.MCBMiniConstants.Channel;

/**
 * @author siggi
 * @date Mar 27, 2013
 */
public abstract class TargetFunction {

	protected long startTime;
	protected boolean isInitialized;
	protected MCBMiniBoard board;
	protected Channel channel;

	public void setOwner(MCBMiniBoard board, Channel channel){
		this.board = board;
		this.channel = channel;
	}

	public void terminateFunction(){
		if( board != null ){
			board.setTargetFunction(channel, null);
		}
	}

	public void initialize(){
		startTime = System.currentTimeMillis();
		initializeInternal();
	}

	protected abstract void initializeInternal();
	public abstract int applyFunction();
}
