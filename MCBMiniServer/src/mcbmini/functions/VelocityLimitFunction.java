package mcbmini.functions;


/**
 * @author siggi
 * @date Mar 27, 2013
 */
public class VelocityLimitFunction extends TargetFunction{

	private int targetPos;
	private double ticksPerMs;
	private int startPos;

	public VelocityLimitFunction(int targetPosTick, double ticksPerMs) {
		super();
		if( ticksPerMs <= 0 ) throw new RuntimeException("ticksPerMs must be > 0");
		this.targetPos = targetPosTick;
		this.ticksPerMs = ticksPerMs;
	}

	@Override
	protected void initializeInternal() {
		this.startPos = board.getActualTick(channel);
		if( targetPos < startPos ) ticksPerMs *= -1;
	}

	@Override
	public int applyFunction() {
		long elapsedTime = System.currentTimeMillis() - startTime;

		double target = startPos + elapsedTime * ticksPerMs;
		if( 	(ticksPerMs < 0 && target < targetPos) ||
				(ticksPerMs > 0 && target > targetPos)	){

			terminateFunction();
			return targetPos;
		}
		else{
			return (int) target;
		}
	}
}
