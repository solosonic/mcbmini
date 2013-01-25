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

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.util.ArrayList;
import java.util.prefs.Preferences;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.Timer;

import mcbmini.MCBMiniBoard;
import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniGUI;
import mcbmini.MCBMiniGUI.Box;
import mcbmini.MCBMiniServer;

import processing.core.PApplet;

/**
 * @author siggi
 * @date Dec 11, 2012
 */
public class PlottingWindow extends JFrame{

	private MCBMiniServer server;
	private Plot plot;

	private ArrayList<BoardContainer> boards;

	public PlottingWindow(MCBMiniServer server, int width, int height){
		super("MCBMini Plotting");
		this.server = server;

		boards = new ArrayList<PlottingWindow.BoardContainer>();
		for (MCBMiniBoard b : server.getBoards()) {
			boards.add( new BoardContainer(b) );
		}

		JPanel panel = new JPanel(new GridBagLayout());
		add(panel);

		plot = new Plot(width, height);
		plot.frame = this;

		GridBagConstraints c = new GridBagConstraints();
		c.anchor = GridBagConstraints.WEST;
		c.weighty = 1;
		c.gridx = 0;
		c.gridy = 0;
		panel.add( plot, c );

		c.gridx++;


		JPanel boardpanel = new JPanel(new GridBagLayout());
		panel.add(boardpanel);
		c.gridx = 0;

		for (BoardContainer b : boards) {
			Box box = new Box("Board "+b.channelA.board.getId());
			boardpanel.add( box, c );
			c.gridy++;
			Box boxA = new Box("Channel A");
			box.addView( boxA);
			boxA.addView(b.channelA.target);
			boxA.addViewToRight(b.channelA.actual);
			Box boxB = new Box("Channel B");
			box.addView( boxB );
			boxB.addView(b.channelB.target);
			boxB.addViewToRight(b.channelB.actual);
		}

		JButton reset = new JButton("Reset Axis");
		reset.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				plot.clearData();
			}
		});
		boardpanel.add( reset, c );
		c.gridy++;

		pack();

		final Preferences prefs = Preferences.userNodeForPackage( PlottingWindow.class );

		String x_str = prefs.get("MCBMiniPlotting_x", null);
		String y_str = prefs.get("MCBMiniPlotting_y", null);
		if( x_str != null && y_str != null ){
			setLocation(Integer.valueOf(x_str), Integer.valueOf(y_str));
		}


		addComponentListener(new ComponentListener() {
			//@Override
			public void componentShown(ComponentEvent arg0) {}
			//@Override
			public void componentResized(ComponentEvent arg0) {  }
			//@Override
			public void componentHidden(ComponentEvent arg0) { }

			//@Override
			public void componentMoved(ComponentEvent arg0) {
				prefs.putInt("MCBMiniPlotting_x", getX());
				prefs.putInt("MCBMiniPlotting_y", getY());
			}
		});
	}

	public void update(){
		if( isVisible() ){
			for (BoardContainer b: boards) {
				b.channelA.update();
				b.channelB.update();
			}

			plot.draw2();
		}
	}

	private class BoardContainer{
		public ChannelContainer channelA;
		public ChannelContainer channelB;

		public BoardContainer(MCBMiniBoard board){
			channelA = new ChannelContainer(board, Channel.A);
			channelB = new ChannelContainer(board, Channel.B);
		}
	}

	private class ChannelContainer{

		public JCheckBox actual;
		public JCheckBox target;

		public int last_target;
		public MCBMiniBoard board;
		public Channel channel;

		public ChannelContainer(MCBMiniBoard board, Channel channel){
			this.board = board;
			this.channel = channel;
			actual = new JCheckBox("Actual");
			target = new JCheckBox("Target");
		}

		public void update(){
			if( target.isSelected() ){
				plot.addDataPoint("Board: "+board.getId()+" ch "+channel+": Target", last_target);
				last_target = board.getTargetTick(channel);
			}
			if( actual.isSelected() ){
				plot.addDataPoint("Board: "+board.getId()+" ch "+channel+": Actual", board.getActualTick(channel));
			}
		}
	}
}
