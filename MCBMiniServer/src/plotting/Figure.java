/*
  A simple Processing library to provide an easy access to
  plotting various data in the main window. One-dimensional
  and two-dimensional scatter plots with various interpolation
  options.

  (c) Roman Krashanitsa

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
 */

package plotting;


import java.awt.Rectangle;
import java.util.Arrays;

import processing.core.PApplet;
import processing.core.PFont;
import processing.core.PGraphicsJava2D;
import processing.core.PImage;


/**
 * A class representing a visual figure object.
 *
 * For example:
 * <pre>
 *    Figure fig = new Figure();
 * </pre>
 *
 * @author  Roman Krashanitsa
 * @version %I%, %G%
 * @see     processing.core.PApplet
 */
public class Figure {


	//Public fields

	public final String VERSION = "0.1.0";

	/**
	 * Autoscale along the X axis. A value of <code>true</code> will cause Figure to autoscale the X axis.
	 */
	public boolean autoScaleX = false;
	/**
	 * Autoscale along the Y axis. A value of <code>true</code> will cause Figure to autoscale the Y axis.
	 */
	public boolean autoScaleY = false;
	/**
	 * A value of <code>true</code> will cause Figure to fill the area of the curve.
	 * If the curve is not closed, it will add one segment between the end-points.
	 */
	public boolean isFilled = false;
	/**
	 * The color of the grid lines.
	 */
	public int gridColor = 0x55FFFFFF;
	/**
	 * The color of the text labels.
	 */
	public int textColor = 0xFFFFFFFF;
	/**
	 * The color of the axial lines.
	 */
	public int axisColor = 0xFFFFFFFF;
	/**
	 * The color of the plot segments.
	 */
	public int lineColor = 0xFFFFFFFF;
	/**
	 * The color of the field background.
	 */
	public int bgColor = 0xFF000000;
	/**
	 * Four values of the array define limits along X and Y axes, respectively.
	 */
	public double _axis[]={0.,100.,0.,1.};
	/**
	 * The thickness of the axis lines.
	 */
	public float axisThickness = 2.0f;
	/**
	 * The thickness of the grid lines.
	 */
	public float gridThickness = 1.0f;
	/**
	 * The thickness of the plot line of the next plot to be drawn.
	 */
	public float lineThickness = 2.0f;
	/**
	 * The thickness of the tick mark line.
	 */
	public float tickThickness = 1.0f;
	/**
	 * The number formatting string.
	 */
	public String numberFormat = "%3.0f";
	/**
	 * Space in pixels between the figure field and the bounding box. Is usually used for axis tick marks and labels.
	 */
	public int _marginX_left = 75;
	public int _marginX_right = 10;

	/**
	 * Space in pixels between the figure field and the bounding box. Is usually used for axis tick marks and labels.
	 */
	public int _marginY = 30;



	//Private fields

	//reference to the parent PApplet object
	private PApplet field;

	//bounding box
	private Rectangle bnd;


	//not used and should be deleted
	private PImage bg;
	//not used and should be deleted
	private PGraphicsJava2D g;

	//scale of the data along X axis to pixels
	private double scalex;

	//scale of the data along y axis to pixels
	private double scaley;

	//tick spacing along x axis
	private double stepX;
	//tick spacing along y axis
	private double stepY;

	//x component of the point of intersection of x and y axes
	private long _x0 = 0;
	//y component of the point of intersection of x and y axes
	private long _y0 = 0;

	//span of displayed data along x axis
	private double _spanX=2;
	//span of displayed data along y axis
	private double _spanY=100;

	//initialized font object used for all text operations
	private PFont font;



	/**
	 * Constructor of the Figure object.
	 *
	 * @param     theParent  the parent object.
	 * @param     bnd  the bounding box of the figure.
	 * @see       processing.core.PApplet
	 * @see       java.awt.Rectangle
	 */
	public Figure(PApplet theParent,Rectangle bnd) {
		field = theParent;
		this.bnd = bnd;
		font = field.createFont("Arial", 10, false);
		field.textFont(font);

		update();
	}


	/**
	 * Displays grid lines using current settings
	 */
	public void drawGrid() {

		field.stroke(gridColor);
		field.strokeWeight(gridThickness);
		//+side
		long x=_x0+bnd.x+_marginX_left;
		double lx=_axis[1]-_spanX;
		while (x<bnd.x+bnd.width-_marginX_right) {
			field.line(x,bnd.y+bnd.height-_marginY,x,bnd.y+_marginY);
			lx+=stepX;
			x+=stepX*scalex;
		}
		//- side
		x=_x0+bnd.x+_marginX_left-Math.round(stepX*scalex);
		lx=_axis[1]-_spanX-stepX;
		while (x>_marginX_left+bnd.x) {
			field.line(x,bnd.y+bnd.height-_marginY,x,bnd.y+_marginY);
			lx-=stepX;
			x-=stepX*scalex;
		}

		//+side
		long y=bnd.y+bnd.height-(_y0+_marginY);
		double ly = _axis[3]-_spanY;
		while (y>bnd.y+_marginY) {
			field.line(bnd.x+_marginX_left,y,bnd.x+bnd.width-_marginX_right,y);
			ly+=stepY;
			y-=stepY*scaley;
		}
		//- side
		y=bnd.y+bnd.height-(_y0+_marginY-Math.round(stepY*scaley));
		ly=_axis[3]-_spanY-stepY;
		while (y<bnd.y+bnd.height-_marginY) {
			field.line(bnd.x+_marginX_left,y,bnd.x+bnd.width-_marginX_right,y);
			ly-=stepY;
			y+=stepY*scaley;
		}
	}


	/**
	 * Displays tick mark lines and labels using current settings
	 */
	public void drawTicks() {

		field.fill(textColor);
		field.stroke(axisColor);
		field.strokeWeight(tickThickness);
		String label;
		//+side
		long x=_x0+bnd.x+_marginX_left;
		double lx=_axis[1]-_spanX;
		while (x<bnd.x+bnd.width-_marginX_right) {
			field.line(x,bnd.y+bnd.height-_marginY-_y0,x,bnd.y+bnd.height-_marginY-_y0-5);
			label=String.format(numberFormat, lx);
			field.text(label,x-field.textWidth(label)/2,bnd.y+bnd.height-_marginY-_y0+3+field.textAscent());
			lx+=stepX;
			x+=stepX*scalex;
		}
		//- side
		x=_x0+bnd.x+_marginX_left-Math.round(stepX*scalex);
		lx=_axis[1]-_spanX-stepX;
		while (x>_marginX_left+bnd.x) {
			field.line(x,bnd.y+bnd.height-_marginY-_y0,x,bnd.y+bnd.height-_marginY-_y0-5);
			label=String.format(numberFormat, lx);
			field.text(label,x-field.textWidth(label)/2,bnd.y+bnd.height-_marginY-_y0+3+field.textAscent());
			lx-=stepX;
			x-=stepX*scalex;
		}

		//+side
		long y=bnd.y+bnd.height-(_y0+_marginY);
		double ly = _axis[3]-_spanY;
		while (y>bnd.y+_marginY) {
			field.line(bnd.x+_marginX_left+_x0,y,bnd.x+_marginX_left+_x0+5,y);
			label=String.format(numberFormat, ly);
			field.text(label,bnd.x+_marginX_left+_x0-5-field.textWidth(label),y+(field.textAscent())/4);
			ly+=stepY;
			y-=stepY*scaley;
		}
		//- side
		y=bnd.y+bnd.height-(_y0+_marginY-Math.round(stepY*scaley));
		ly=_axis[3]-_spanY-stepY;
		while (y<bnd.y+bnd.height-_marginY) {
			field.line(bnd.x+_marginX_left+_x0,y,bnd.x+_marginX_left+_x0+5,y);
			label=String.format(numberFormat, ly);
			field.text(label,bnd.x+_marginX_left+_x0-5-field.textWidth(label),y+field.textAscent());
			ly-=stepY;
			y+=stepY*scaley;
		}

	}


	/**
	 * Displays axes using current settings
	 */
	public void drawAxes() {
		field.stroke(axisColor);
		field.strokeWeight(axisThickness);
		//draw axes
		field.line(bnd.x+_marginX_left,bnd.y+bnd.height-_marginY-_y0,bnd.x+bnd.width-_marginX_right,bnd.y+bnd.height-_marginY-_y0);
		field.line(bnd.x+_marginX_left+_x0,bnd.y+bnd.height-_marginY,bnd.x+_marginX_left+_x0,_marginY+bnd.y);
	}

	/**
	 * Updates all internal scales and settings for the Figure. Needs to be called when auto-scale or axis settings are changed.
	 */

	public void update() {
		_spanX = _axis[1]-_axis[0];
		_spanY = _axis[3]-_axis[2];
		_x0 = Math.round((_axis[0]*_axis[1]<0)?bnd.width*(_axis[0]/_spanX):((_axis[1]<0)?bnd.width:0));
//		_y0 = Math.round((_axis[2]*_axis[3]<0)?bnd.height*(_axis[2]/_spanY):((_axis[3]<0)?bnd.height:0));

		if( _axis[2] > 0 && _axis[3] > 0 ){
			_y0 = Math.round(bnd.height*(_axis[2]/_spanY));
		}
		else if( _axis[2] < 0 && _axis[3] < 0 ){
			_y0 = bnd.height;
		}
		else if( _axis[2] < 0 && _axis[3] > 0 ){
			_y0 = 0;
		}

		int n=10;
		long nn=Math.round(Math.log(_spanX)/Math.log(10));
		double sc=Math.pow(10,nn);
		double step=_spanX/sc/n;
		if (step>0.12) step=0.15;
		else if (step>0.07) step=0.1;
		else if (step>0.03) step=0.05;
		else step=0.02;

		stepX=step*sc;

		n=10;
		nn=Math.round(Math.log(_spanY)/Math.log(10));
		sc=Math.pow(10,nn);
		step=_spanY/sc/n;
		if (step>0.12) step=0.15;
		else if (step>0.07) step=0.1;
		else if (step>0.03) step=0.05;
		else step=0.02;

		stepY=step*sc;
		//	    PApplet.println("nn="+nn+", sc="+sc+", stepY="+stepY);

		if (Math.abs(_spanX)>1e-15) scalex=(bnd.width-_marginX_left-_marginX_right)/_spanX;
		else scalex=1.0;
		if (Math.abs(_spanY)>1e-15) scaley=(bnd.height-2*_marginY)/_spanY;
		else scaley=1.0;
		//	    PApplet.println("nn="+nn+", sc="+sc+", stepY="+stepY+", scalex="+scalex+", scaley="+scaley);

	}

	private double[] minmax(double[] data) {

		double[] ret = new double[2];
		ret[0]=data[0];
		ret[1]=data[1];
		for (int j=0;j<data.length;j++) {
			if (data[j] < ret[0]) ret[0] = data[j];
			if (data[j] > ret[1]) ret[1] = data[j];
		}
		return ret;
	}

	private void rescale(double[] data){

		boolean upd = false;
		if (autoScaleX) {
			_spanX = data.length-1;
			_axis[0]=0;
			_axis[1]=_spanX;
			upd=true;
		}
		if (autoScaleY) {
			double[] min_max=minmax(data);
			_axis[2] = Math.min(_axis[2], min_max[0]);
			_axis[3] = Math.max(_axis[3], min_max[1]);
			_spanY=(_axis[3]-_axis[2]);
			//			PApplet.println(min_max[0]+","+min_max[1]);
			_spanY*=1.1;
			//			PApplet.println("_spanY="+_spanY+", _y0="+_y0);
			upd=true;
		}
		if (upd) {
			update();
		}
	}

	private void rescale(double[] x, double[] y){

		boolean upd = false;
		if (autoScaleX) {
			double[] min_max=minmax(x);
			_axis[0] = min_max[0];
			_axis[1] = min_max[1];
			_spanX=(_axis[1]-_axis[0]);
			//			PApplet.println(min_max[0]+","+min_max[1]);
			_spanX*=1.1;
			//			PApplet.println("_spanY="+_spanY+", _y0="+_y0);
			upd=true;
		}
		if (autoScaleY) {
			double[] min_max=minmax(y);
			_axis[2] = Math.min(_axis[2], min_max[0]);
			_axis[3] = Math.max(_axis[3], min_max[1]);
			_spanY=(_axis[3]-_axis[2]);
			//			PApplet.println(min_max[0]+","+min_max[1]);
			_y0 = Math.round((_axis[2]*_axis[3]<0)?bnd.height*(min_max[0]/_spanY):((min_max[1]<0)?bnd.height:0));
			_spanY*=1.1;
			//			PApplet.println("_spanY="+_spanY+", _y0="+_y0);
			upd=true;
		}
		if (upd) {
			update();
		}
	}


	/**
	 * Plot data.
	 *
	 * @param     data  a one-dimensional array of data values.
	 * @param     approx  the method of approximation between the points (@see plotting.Figure.Approx).
	 * @see       plotting.Figure.Approx
	 */
	public void plot(double[] data, Approx approx) {
		plot(data, approx, 0, data.length);
	}

	/**
	 * Plot data.
	 *
	 * @param     data  a one-dimensional array of data values.
	 * @param     approx  the method of approximation between the points (@see plotting.Figure.Approx).
	 * @param     start start plotting at this element of the <code>data</code> array.
	 * @param     num plot this many elements of the <code>data</code> array.
	 * @see       plotting.Figure.Approx
	 */
	public void plot(double[] data, Approx approx, int start, int num ) {

		rescale(data); //check if AutoScale is on

		field.stroke(lineColor);
		field.strokeWeight(lineThickness);
		field.strokeCap(PApplet.ROUND);
		field.strokeJoin(PApplet.BEVEL);

		int n=0;
		int i = start;
		int i_1;
		if (!this.isFilled) field.noFill();

		field.beginShape();
		field.vertex( bnd.x+_marginX_left + _x0 + Math.round( (n - 0) * scalex),
				bnd.y+bnd.height - (_marginY + _y0 + Math.round( (data[i]-_axis[2]) * scaley)));

		i_1=i;
		i+=1; if (i==data.length) i=0;

		switch (approx) {
		case nearest:
			for (n = 1; n < num; n++) {
				field.vertex(bnd.x + _marginX_left + _x0 + Math.round((n - 0) * scalex),
						bnd.y + bnd.height - (_marginY + _y0 + Math.round((data[i_1]-_axis[2]) * scaley)));
				field.vertex(bnd.x + _marginX_left + _x0	+ Math.round((n - 0) * scalex),
						bnd.y + bnd.height	- (_marginY + _y0 + Math.round((data[i]-_axis[2]) * scaley)));
				i_1=i;
				i+=1; if (i==data.length) i=0;
			}
			break;
		case linear:
			for (n = 1; n < num; n++) {
				field.vertex(bnd.x + _marginX_left + _x0	+ Math.round(n * scalex),
						bnd.y + bnd.height	- (_marginY + _y0 + Math.round((data[i]-_axis[2]) * scaley)));
				i+=1; if (i==data.length) i=0;
			}
			break;

		}


		field.endShape();

	}

	/**
	 * Plot data.
	 *
	 * @param     x  a one-dimensional array of x values.
	 * @param     y  a one-dimensional array of y values.
	 * @param     approx  the method of approximation between the points (@see plotting.Figure.Approx).
	 * @see       plotting.Figure.Approx
	 */
	public void plot(double[] x, double[] y, Approx approx) {
		plot(x, y, approx, 0, x.length);
	}

	/**
	 * Plot data.
	 *
	 * @param     x  a one-dimensional array of x values.
	 * @param     y  a one-dimensional array of y values.
	 * @param     approx  the method of approximation between the points (@see plotting.Figure.Approx).
	 * @param     start start plotting at this element of the supplied points array.
	 * @param     num plot this many elements of the the supplied points array.
	 * @see       plotting.Figure.Approx
	 */
	public void plot(double[] x, double[] y, Approx approx, int start, int num) {

		if (x.length!=y.length) throw new java.lang.ArrayIndexOutOfBoundsException("Lengths of x and y should be same.");

		rescale(x,y); //check if AutoScale is on

		field.stroke(lineColor);
		field.strokeWeight(lineThickness);
		field.strokeCap(PApplet.ROUND);
		field.strokeJoin(PApplet.BEVEL);
		int n=0;
		int i = start;

		if (!this.isFilled) field.noFill();

		field.beginShape();
		field.vertex( bnd.x+_marginX_left + _x0 + Math.round( (x[i]-_axis[0]) * scalex),
				bnd.y+bnd.height - (_marginY + _y0 + Math.round( (y[i]-_axis[2]) * scaley)));

		i+=1; if (i==x.length) i=0;

		switch (approx) {
		case linear:
			for (n = 1; n < num; n++) {
				field.vertex(bnd.x + _marginX_left + _x0	+ Math.round((x[i]-_axis[0]) * scalex),
						bnd.y + bnd.height	- (_marginY + _y0 + Math.round((y[i]-_axis[2]) * scaley)));
				i+=1; if (i==x.length) i=0;
			}
			break;
		case nearest:
			throw new IllegalArgumentException("Can not use Approx.nearest in scatter plot.");

		}


		field.endShape();

	}

	public void draw() {
		field.fill(bgColor);
		field.stroke(bgColor);
		field.rect(bnd.x, bnd.y, bnd.width, bnd.height);
		field.noFill();

		drawGrid();
		drawAxes();
		drawTicks();
	}


	/**
	 * A type representing a type of approximation used in plotting.
	 */
	public enum Approx {nearest,linear,quadratic,cubic};


	/**
	 * return the version of the library.
	 *
	 * @return String
	 */
	public String version() {
		return VERSION;
	}

}


