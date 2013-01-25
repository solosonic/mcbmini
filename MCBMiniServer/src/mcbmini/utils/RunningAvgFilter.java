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

public class RunningAvgFilter {
	protected float value;
	protected float filter_order_primary;
	protected float filter_order_stored;
	protected int value_cnt;
	protected float large_ratio, small_ratio, min_val, max_val;
	protected boolean has_been_initialized = false;

	public RunningAvgFilter(float order, float initial_val, float limit1, float limit2){
		if( initial_val == Float.MAX_VALUE ){
			has_been_initialized = false;
		}
		else{
			value = initial_val;
			has_been_initialized = true;
		}
		value_cnt = 1;
		filter_order_primary = order;
		this.min_val = (limit1<limit2)?limit1:limit2;
		this.max_val = (limit1>limit2)?limit1:limit2;

	}
	public float resetFilter(float reset_to_value){
		if( reset_to_value<min_val ) reset_to_value = min_val;
		if( reset_to_value>max_val) reset_to_value = max_val;

		value = reset_to_value;
		return value;
	}

	public void changeFiltOrder(float order){
		filter_order_primary = order;
	}

	public float readVal(){
		return value;
	}

	public float updateVal(float in){
		if( in<min_val ) in = min_val;
		if( in>max_val) in = max_val;

		if( !has_been_initialized ){
			has_been_initialized = true;
			value = in;
		}

		large_ratio = (float)(filter_order_primary-1)/filter_order_primary;


		value = large_ratio*value + (1-large_ratio)*in;
		return value;
	}
}
