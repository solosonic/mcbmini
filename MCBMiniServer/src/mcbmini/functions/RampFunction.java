package mcbmini.functions;

import javax.management.RuntimeErrorException;

/**
 * @author siggi
 * @date Mar 27, 2013
 */
public class RampFunction extends TargetFunction{

	private int targetPos;
	private int durationMs;
	private long endTime;
	private double slope;
	private double intersect;

	public RampFunction(int targetPosTick, int durationMs) {
		super();
		if( durationMs <= 0 ) throw new RuntimeException("Duration must be > 0");
		this.targetPos = targetPosTick;
		this.durationMs = durationMs;
	}

	@Override
	protected void initializeInternal() {
		this.endTime = startTime + durationMs;
		slope = (double)(targetPos - board.getActualTick(channel)) / (double) durationMs;
		intersect = targetPos - slope * endTime;
	}

	@Override
	public int applyFunction() {
		long time = System.currentTimeMillis();
		if( time > endTime ){
			terminateFunction();
			return targetPos;
		}

		return (int)( slope * time + intersect );
	}
}
