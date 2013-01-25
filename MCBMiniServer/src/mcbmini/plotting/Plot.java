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
 
package mcbmini.plotting;

import java.awt.Dimension;
import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import processing.core.PApplet;
import plotting.*;

/**
 * @author siggi
 * @date Dec 11, 2012
 */
public class Plot extends PApplet{

	public static final int BUFFER_LENGTH = 50;
	public static final int legend_y = 20;
	public static final int legend_x = 100;
	public static final int legend_width = 150;

	private Figure fig;
	private Map<String, DataLine> data;
	private int w_, h_;

	public Plot() {
		this(640, 480);
	}


	public Plot(int width, int height) {
		w_ = width;
		h_ = height;
		data = new HashMap<String, Plot.DataLine>();

	    this.finished = false;
	    this.looping = true;
	    this.redraw = true;
		this.g = makeGraphics(w_, h_, JAVA2D, null, true);
		this.width = g.width;
		this.height = g.height;

		setSize(w_, h_);
		setPreferredSize(new Dimension(w_, h_));

		fig = new Figure(this, new Rectangle(0, 0, width, height));
		fig.bgColor = color(255);
		fig.axisColor = color(0);
		fig.textColor = color(0);
		fig.gridColor = color(200);

		fig.numberFormat = "%3.1f";
		fig.autoScaleX=true;
		fig.autoScaleY=true;
		fig.update();

		fig.isFilled=false;
		fig.lineThickness = 2.0f;
	}

	public void setup(){
		size(w_, h_, JAVA2D);
	}

	public void clearData(){
		data.clear();
		fig._axis[0] = 0;
		fig._axis[1] = 1;
		fig._axis[2] = 0;
		fig._axis[3] = 1;
	}

	public void addDataPoint(String name, double y){
		DataLine dataLine = data.get(name);
		if( dataLine == null ){
			dataLine = new DataLine(name);
			data.put(name, dataLine);
		}
		dataLine.addData(y);
	}

	public void resize(){
		Dimension currentSize = new Dimension();
		getSize(currentSize);

		if (currentSize.width != g.width || currentSize.height != g.height) {
			resizeRenderer(currentSize.width, currentSize.height);
		}
	}

	boolean has_resized = false;
	public void draw2(){
	    g.beginDraw();

		if( !has_resized ){
			resize();
			setPreferredSize(getSize());
			has_resized = true;
		}
		noFill();
		background(255);

		fig.draw();

		for (DataLine d : data.values()) {
			fig.lineColor = d.color;
			fig.plot(d.y, Figure.Approx.linear, d.index, d.y.length);
		}

		int y = legend_y;
		strokeWeight(1);
		fill(color(200));
		rect(legend_x, y-10, legend_width, data.values().size() * 15 );

		fill(color(0));
		strokeWeight(4);
		for (DataLine d : data.values()) {
			stroke(d.color);
			line(legend_x+5, y-3, legend_x+25, y-3);
			text(d.name, legend_x+40, y);
			y += 15;
		}

	    g.endDraw();
	    paint();
	}

	private class DataLine{
		public String name;
		protected int color;
		protected double[] y;
		protected int index;

		public DataLine(String name) {
			this.name = name;
			this.color = color(random(150), random(150), random(150));
			y = new double[BUFFER_LENGTH];
			index = 0;
		}

		public void addData(double y){
			this.y[ index ] = y;
			index = (index+1) % this.y.length;
		}
	}

}
