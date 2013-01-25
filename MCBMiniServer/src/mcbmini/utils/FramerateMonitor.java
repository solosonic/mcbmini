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

import java.util.Timer;
import java.util.TimerTask;

/**
 * add one of these to the update list of a system whose framerate you want to monitor.
 *
 * @author mattb
 */
public class FramerateMonitor
{
	protected Timer timer;
	protected boolean shouldDisplayText;
	protected String prefixText;

	public FramerateMonitor(float updateInterval)
	{
		this(updateInterval, "");
		shouldDisplayText = false;
	}

	public FramerateMonitor(float updateInterval, String prefixText)
	{
		this.prefixText = prefixText;
		this.shouldDisplayText = true;
		timer = new Timer("FramerateMonitorTimer."+prefixText);
		timer.schedule(new TimerTask(){
			//@Override
			public void run(){
				updateFPS();
			}
		}, Math.round(updateInterval * 1000f), Math.round(updateInterval * 1000f));
	}

	protected int tickCount = 0;
	protected long startTime = System.currentTimeMillis();
	protected float fps = 0;

	public synchronized void update()
	{
		tickCount++;
	}

	protected void updateFPS()
	{
		long endTime = System.currentTimeMillis();
		synchronized (this) {
			fps = tickCount / ((endTime-startTime)/1000f);
			tickCount = 0;
		}

		startTime = endTime;

		if (shouldDisplayText)
		{
			System.err.println(prefixText+fps);
		}
	}

	/**
	 * Returns the framerate of this monitor
	 * This method is thread safe with the update()  (they can be called from different threads)
	 * @return
	 */
	public float getFPS()
	{
		float ret;
		synchronized (this) {
			ret = fps;
		}
		return ret;
	}
}
