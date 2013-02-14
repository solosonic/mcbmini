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

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JSlider;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.ScrollPaneConstants;
import javax.swing.Timer;
import javax.swing.UIManager;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import javax.swing.filechooser.FileFilter;

import mcbmini.MCBMiniConstants.Error;
import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.Command;
import mcbmini.MCBMiniConstants.ExtraPinMode;
import mcbmini.MCBMiniConstants.StreamMode;
import mcbmini.MCBMiniConstants.FeedbackMode;
import mcbmini.MCBMiniConstants.MotorPolarity;
import mcbmini.MCBMiniConstants.ControlMode;
import mcbmini.MCBMiniServer.MCBMiniBoardDisabledHandler;
import mcbmini.MCBMiniServer.MCBMiniIDResponseHandler;
import mcbmini.MCBMiniServer.MCBMiniResponseHandler;
import mcbmini.plotting.PlottingWindow;
import mcbmini.serial.MCBMiniNativeLoader;
import mcbmini.utils.RunningAvgFilter;
import mcbmini.utils.XMLUtils;
import mcbmini.utils.XMLUtils.XMLResults;

import java.awt.Component;
import java.awt.FlowLayout;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.FocusEvent;
import java.awt.event.FocusListener;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.prefs.Preferences;

public class MCBMiniGUI {

	static{
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private static Preferences prefs = Preferences.userNodeForPackage( MCBMiniGUI.class );
	private static String SEP = System.getProperty("file.separator");

	private static MCBMiniServer server_instance;

	private static void printHelp(){
		System.out.println("Usage: java -jar MCBMini.jar [options]");
		System.out.println("options:");
		System.out.println("\t-help (printout this list of help options)");
		System.out.println("\t-config config_file.xml (required)");
		System.out.println("\t-debug (run without a serial connection)");
		System.out.println("\t-port port_name (to override port in xml file)");
		System.out.println("\t-lib path_to_lib (path to native rxtx libraries)");
	}

	private static void error(String error){
		System.err.println(error);
		String[] options = new String[]{"OK"};
		JOptionPane.showOptionDialog(null,
			    error,
			    "Error",
			    JOptionPane.YES_OPTION,
			    JOptionPane.ERROR_MESSAGE,
			    null,
			    options,
			    options[0]);
	}

	public static void main(String[] args) {
		MCBMiniGUI gui = create(args);
		gui.setTargetSlidersActiveControl(true);
	}

	public static MCBMiniGUI create(String[] args) {
//		args = new String[]{"-config", "test.xml"};

		String xml_file = null;
		boolean debug = false;
		String port_name = null;

		/*
		 * If no arguments are passed, we might just have gotten clicked on so we open a choose file dialog
		 */
		if( args.length == 0 ){
			printHelp();
		}
		else{
			/*
			 * Here we parse the command line options
			 */

			for(int i=0; i<args.length; i++){
				if( args[i].equals("-help") ){
					printHelp();
					System.exit(0);
				}
				if( args[i].equals("-debug") ){
					debug = true;
					System.out.println("Running in debug mode (without a serial connection)");
				}
				if( args[i].equals("-port") && args.length > i+1 ){
					port_name = args[i+1];
					System.out.println("Overriding port name in config file with: "+port_name);
				}
				if( args[i].equals("-config") && args.length > i+1 ){
					if( new File(args[i+1]).exists() ) xml_file = args[i+1];
					else if( new File(System.getProperty("user.dir")+SEP+args[i+1]).exists() ) xml_file = System.getProperty("user.dir")+SEP+args[i+1];
					else{
						error("Config file \""+args[i+1]+"\" cannot be found");
						System.exit(0);
					}
				}
				if( args[i].equals("-lib") && args.length > i+1 ){
					String lib_path = null;
					if( new File(args[i+1]).exists() ) lib_path = args[i+1];
					else if( new File(System.getProperty("user.dir")+SEP+args[i+1]).exists() ) lib_path = System.getProperty("user.dir")+SEP+args[i+1];
					else{
						System.err.println("Library file \""+args[i+1]+"\" cannot be found");
						System.exit(0);
					}
					System.setProperty("rxtx.library.path", lib_path);
					System.out.println("Setting native rxtx library path to \""+lib_path+"\"");
				}
			}

			if( xml_file == null ){
				final JFileChooser fc = new JFileChooser(new File(System.getProperty("user.dir")));
				fc.setFileFilter(new FileFilter() {
					//@Override
					public String getDescription() {
						return "Motor config files";
					}

					//@Override
					public boolean accept(File f) {
						if( f.isDirectory() || f.getName().toLowerCase().endsWith(".xml") ) return true;
						return false;
					}
				});
				fc.setName("Select motor config file");
				int returnVal = fc.showOpenDialog(null);
				if( returnVal == JFileChooser.CANCEL_OPTION || fc.getSelectedFile().isDirectory()){
					System.exit(0);
				}

				xml_file = fc.getSelectedFile().getAbsolutePath();
			}
		}

		if( !debug && MCBMiniNativeLoader.shouldLookForNative() && MCBMiniNativeLoader.findLibraryLocation() == null ){
			error("Can't find native libraries, make sure that the jar is next to the \"lib\" folder");
			System.exit(0);
		}

		XMLResults xml_results = null;
		try {
			xml_results = XMLUtils.parseMCBMiniConfigFile(xml_file);
		} catch (Exception e1) {
			error("Error parsing xml file: "+e1.getMessage());
			System.exit(0);
		}

		if( xml_results.port_name == null && port_name == null && !debug ){
			error("Please specify port name as cmd line option or in config file");
			System.exit(0);
		}

		try {
			if( debug ){
				server_instance = new DebugMCBMiniServer(xml_results.boards);
			}
			else{
				server_instance = new MCBMiniServer( port_name==null?xml_results.port_name:port_name, xml_results.boards);
				if( xml_results.minimum_firmware_version != 0 )server_instance.setMinimumFirmwareVersion(xml_results.minimum_firmware_version);
			}
		} catch (IOException e) {
			error(e.getMessage());
			System.exit(0);
		}

		final MCBMiniGUI gui = new MCBMiniGUI(server_instance);

		// Main timer that updates GUI and event handling
		float upd_frequency = 30;
		Timer t = new Timer((int)(1000/upd_frequency), new ActionListener() {
//			//@Override
			public void actionPerformed(ActionEvent arg0) {
				server_instance.update();
				gui.update();
			}
		});
		t.start();

		return gui;
	}


	private MCBMiniServer server;
	private ArrayList<SetTargetPositionElement> set_target_pos_elements;
	private ArrayList<ErrorElement> error_elements;

	private HashMap<Integer, MCBMiniBoard> boards_by_id;

	private JCheckBox should_use_target_pos;
	private JLabel internal_update_fps;
	private JLabel all_board_fps;

	private Window main_window = new Window("MCBMini Motors Interface");

	private String[] board_id_list;

	private JLabel bad_checksum_counter;

	public MCBMiniGUI(MCBMiniServer server){
		this.server = server;

		boards_by_id = new HashMap<Integer, MCBMiniBoard>();
		board_id_list = new String[server.getBoards().size()];
		int cnt = 0;
		for (MCBMiniBoard mcbMiniBoard : server.getBoards()) {
			boards_by_id.put( Integer.valueOf(mcbMiniBoard.getId()) , mcbMiniBoard);
			board_id_list[cnt++] = mcbMiniBoard.getId()+"";
		}

		set_target_pos_elements = new ArrayList<SetTargetPositionElement>();
		error_elements = new ArrayList<MCBMiniGUI.ErrorElement>();

		should_use_target_pos = new JCheckBox("Enable This Interface", true);
		should_use_target_pos.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if( should_use_target_pos.isSelected() ){
					for (SetTargetPositionElement el : set_target_pos_elements) {
						el.target_pos[0].grayOut(false);
						el.enable[0].setEnabled(true);
						el.target_pos[1].grayOut(false);
						el.enable[1].setEnabled(true);
					}
				}
				else{
					for (SetTargetPositionElement el : set_target_pos_elements) {
						el.target_pos[0].grayOut(true);
						el.enable[0].setEnabled(false);
						el.target_pos[1].grayOut(true);
						el.enable[1].setEnabled(false);
					}
				}
			}
		});

		// Here we register for events when the motor server disables boards behind our backs
		server.addBoardDisabledEventHandler(new MCBMiniBoardDisabledHandler() {
			public void handleBoardDisableEvent(MCBMiniBoard board, Channel ch) {
				for (SetTargetPositionElement el : set_target_pos_elements) {
					if( el.motor == board ){
						int channel = ch==Channel.A?0:1;
						el.enable[channel].setSelected(false);
					}
				}
			}
		});

		Box mainpanel = new Box();
		Box parameterpanel = new Box();

		Box errorpanel = new Box();


		Box topbox = new Box();
		Box box = new Box("Enable Interface");
		box.addView(should_use_target_pos);
		topbox.addView(box);
		internal_update_fps = new JLabel("Internal update rate: 00.0 Hz");
		all_board_fps = new JLabel("Board cycle update rate: 00.0 Hz");
		box = new Box("FPS");
		box.addView(internal_update_fps);
		box.addView(all_board_fps);
		topbox.addViewToRight(box);

		JButton plotting_btn = new JButton("Open plotting window");
		plotting_btn.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent arg0) {
				plottingwindow.setVisible(true);
			}
		});
		topbox.addViewToRight(plotting_btn);


		mainpanel.addView(topbox);

		/*
		 * Create the set targets and parameters and error window
		 */
		bad_checksum_counter = new JLabel("Nr. bad checksums received: "+0+"        ");
		errorpanel.addView(bad_checksum_counter);

		Iterator<MCBMiniBoard> ii = server.getBoards().iterator();
		while( ii.hasNext() ){
			MCBMiniBoard b = ii.next();
			SetParametersElement el = new SetParametersElement(b);
			ErrorElement el2 = new ErrorElement(b);
			SetTargetPositionElement el3 = new SetTargetPositionElement(b);

			parameterpanel.addView( el.init() );
			errorpanel.addView( el2.init() );
			mainpanel.addView( el3.init() );

			error_elements.add( el2 );
			set_target_pos_elements.add( el3 );
		}


		JScrollPane scroll = new JScrollPane(mainpanel);
		scroll.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		main_window.addView("Set Motor Targets", scroll);

		scroll = new JScrollPane(parameterpanel);
		scroll.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		main_window.addView("Set Motor Paramterers", scroll);

		scroll = new JScrollPane(errorpanel);
		scroll.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		main_window.addView("Reported Errors", scroll);

		scroll = new JScrollPane(new RequestResponseWindow());
		scroll.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		main_window.addView("Request response", scroll);


		scroll = new JScrollPane(new IDWindow());
		scroll.setHorizontalScrollBarPolicy(ScrollPaneConstants.HORIZONTAL_SCROLLBAR_NEVER);
		main_window.addView("Change Board ID", scroll);


		main_window.pack();
		main_window.setSize(main_window.getSize().width+20, Math.min(700, main_window.getSize().height));
		main_window.setResizable(false);
		main_window.setVisible(true);
		main_window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);


		plottingwindow = new PlottingWindow(server, 640, 480);

		// Set the targets to be unselected at first
		if( should_use_target_pos.isSelected() ){
			should_use_target_pos.doClick();
		}
	}

	public void setTargetSlidersActiveControl(boolean active){
		if( should_use_target_pos.isSelected() != active ){
			should_use_target_pos.doClick();
		}
	}

	PlottingWindow plottingwindow;

	public MCBMiniServer getServer(){
		return server;
	}

	private long time_start = -1;
	private long update_counter = 0;
	public void update(){
		update_counter++;
		if(time_start == -1)
			time_start = System.currentTimeMillis();

		float current_time = (float)(System.currentTimeMillis()-time_start)/1000f;


		/*
		 * Update sliders
		 */
		if( main_window.isVisible() ){
			bad_checksum_counter.setText( "Nr. bad checksums received: "+server.getNumberOfBadChecksums()) ;

			for (final SetTargetPositionElement el : set_target_pos_elements) {
				for (Channel ch : Channel.values()) {
					el.actual_pos[ch.index].setValue( el.motor.getChannelParameter(ch, ChannelParameter.ACTUAL_TICK) );
					//					el.motor_current[ch.index].set( el.motor.getChannelParameter(ch, ChannelParameter.MOTOR_CURRENT) );
					el.motor_current[ch.index].setValue( (int)el.motor.getMotorCurrent(ch) );
					if( should_use_target_pos.isSelected() ){
						int target = el.target_pos[ch.index].evaluate();
						target = Math.round( el.target_filter[ch.index].updateVal( target ) );
						el.motor.setTargetTick(ch, target);
					}
					ExtraPinMode extraPinMode = ExtraPinMode.getExtraPinMode(el.motor, ch);
					if( extraPinMode == ExtraPinMode.ANALOG ){
						el.extra_value[ch.index].setValue( el.motor.getExtraPinValue(ch) );
					}
					else if( extraPinMode == ExtraPinMode.SWITCH ){
						el.extra_switch[ch.index].setSelected( el.motor.getExtraPinValue(ch)==1 );
					}
					else if( extraPinMode == ExtraPinMode.SERVO && should_use_target_pos.isSelected() ){
						el.motor.setExtraPinValue(ch, el.extra_value[ch.index].value);
					}
				}
			}

			/*
			 * Update error window
			 */
			for (ErrorElement el : error_elements) {
				el.update();
			}

			/*
			 * Update the plot
			 */
			plottingwindow.update();

			/*
			 * Update fps
			 */
			float[] fps = server.getUpdateRates(null);
			internal_update_fps.setText("Internal update rate: "+Math.round(fps[0])+" Hz");
			all_board_fps.setText("Board cycle update rate: "+Math.round(fps[1])+" Hz");
		}
	}


	private class IDWindow extends Box{
		private JLabel result_ts, result_id;
		private long start;

		public IDWindow(){
			super();
			start = System.currentTimeMillis();

			addView( new JLabel("IMPORTANT: Only have one board connected to the bus !!") );
			final Slider request_board_id = new Slider("New ID", 0, 20, 1);
			addView( request_board_id );
			JButton change_id = new JButton("Change ID of connected board");
			addView( change_id );
			change_id.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					server.broadcastRequest(Channel.A, Command.ID, (int)request_board_id.evaluate());
				}
			});
			JButton request_id = new JButton("Request ID of connected board");
			addView( request_id);
			request_id.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent e) {
					server.broadcastRequestForIDResponse(new MCBMiniIDResponseHandler() {
						@Override
						public void handleIDResponse(int board_id) {
							result_ts.setText("Timestamp: "+(System.currentTimeMillis()-start));
							result_id.setText("ID: "+ board_id);
						}
						@Override
						public void handleTimeout(int board_id) {
							result_ts.setText("Timestamp: "+(System.currentTimeMillis()-start));
							result_id.setText("ID: TIMEOUT");
						}
					});
				}
			});
			addView( new JLabel(" ") );
			addView( result_ts = new JLabel("Timestamp:                                                                     ") );
			addView( result_id = new JLabel("ID:                                                                     ") );
		}
	}

	private class RequestResponseWindow extends Box{

		private JLabel result_cmd;
		private JLabel result_val;
		private JLabel result_id;
		private JLabel result_channel;
		private JLabel result_ts;
		private long start;

		public RequestResponseWindow(){
			super();
			start = System.currentTimeMillis();

			String[] channel_labels = new String[Channel.values().length];
			for(int i=0; i<Channel.values().length; i++){
				channel_labels[i] = Channel.values()[i].toString();
			}

			final RadioButton board_id = new RadioButton("Board ID", board_id_list, board_id_list[0]);
			final RadioButton channel_sel = new RadioButton("Channel", channel_labels, channel_labels[0]);

			// Create the combobox
			final List<Command> commands = new LinkedList<MCBMiniConstants.Command>();
			for(int i=0; i<Command.values().length; i++){
				if(
						Command.values()[i] != Command.EMPTY_RESPONSE &&
						Command.values()[i] != Command.ERROR &&
						Command.values()[i] != Command.REQUEST_MESSAGE &&
						Command.values()[i] != Command.TWO_TARGET_TICK_ACTUAL &&
						Command.values()[i] != Command.TWO_TARGET_TICK_VELOCITY &&
						Command.values()[i] != Command.DEBUG &&
						Command.values()[i] != Command.TWO_TARGET_TICK_MOTOR_CURRENT ){
					commands.add(Command.values()[i]);
				}
			}
			Collections.sort(commands, new Comparator<Command>() {
				@Override
				public int compare(Command o1, Command o2) {
					return o1.toString().compareTo(o2.toString());
				}
			});

			//Create the combo box, select item at index 4.
			//Indices start at 0, so 4 specifies the pig.
			final JComboBox combo = new JComboBox( commands.toArray() );
			combo.setSelectedIndex(0);

			JButton button = new JButton("Request response");
			button.addActionListener(new ActionListener() {
				public void actionPerformed(ActionEvent arg0) {
					MCBMiniBoard board = boards_by_id.get( Integer.parseInt( (String)board_id.evaluate() ));
					Channel channel = Channel.valueOf( (String)channel_sel.evaluate() );
					server.sendRequestForResponse(board, channel, commands.get(combo.getSelectedIndex()), new MCBMiniResponseHandler() {
						@Override
						public void handleResponse(MCBMiniBoard board, Channel channel, Command command, int value) {
							System.out.println("Received response: "+board.getId()+" "+channel+" "+command+" " +value);
							result_ts.setText("Timestamp: "+(System.currentTimeMillis()-start));
							result_id.setText("ID: "+board.getId());
							result_channel.setText("Channel: "+channel);
							result_cmd.setText("Command: "+command);
							result_val.setText("Value: "+value);
						}
					});
				}
			});

			addView( board_id );
			addView( channel_sel );
			addView( combo );
			addView( button );
			addView( new JLabel(" ") );
			addView( result_ts = new JLabel("Timestamp:                  ") );
			addView( result_id = new JLabel("ID:                         ") );
			addView( result_channel = new JLabel("Channel:               ") );
			addView( result_cmd = new JLabel("Command:                        ") );
			addView( result_val = new JLabel("Value:                     ") );
		}
	}

	private class SetTargetPositionElement{
		public RunningAvgFilter[] target_filter = new RunningAvgFilter[2];
		public Slider[] target_pos = new Slider[2];
		public Slider[] actual_pos = new Slider[2];
		public Slider[] motor_current = new Slider[2];
		public JCheckBox[] enable = new JCheckBox[2];

		public JCheckBox[] extra_switch = new JCheckBox[2];
		public Slider[] extra_value = new Slider[2];

		public MCBMiniBoard motor;

		public SetTargetPositionElement(MCBMiniBoard motorboard){
			this.motor = motorboard;
		}

		public Box init(){
			Box view = new Box("MCBMini board "+motor.getId());

			for (final Channel channel : Channel.values()) {
				Box channelbox = new Box("Channel "+channel+":");
				SliderGroup slidergroup = new SliderGroup();
				channelbox.addView( enable[channel.index] = new JCheckBox("Enable", motor.getChannelParameter(channel, ChannelParameter.ENABLED)==1) );
				channelbox.addView(slidergroup);

				int low, high, init;
				if( motor.getChannelParameter(channel, ChannelParameter.FEEDBACK_MODE) == FeedbackMode.POT.id ){
					low = 0; high = 1023; init = 512;
				}
				else{
					if( motor.getChannelParameter(channel, ChannelParameter.CONTROL_MODE) == ControlMode.VELOCITY.id ){
						low = -200; high = 200; init = 0;
					}
					else{
						low = -2000; high = 2000; init = 0;
					}
				}
				target_filter[channel.index] = new RunningAvgFilter(5, init, low, high);
				slidergroup.addView( target_pos[channel.index] = new Slider("Target position", low, high, init) );
				slidergroup.addView( actual_pos[channel.index] = new Slider("Actual position", low, high, init) );
				actual_pos[channel.index].grayOut(true);
				slidergroup.addView( motor_current[channel.index] = new Slider("Motor current", 0, 1024, 0) );
				motor_current[channel.index].grayOut(true);

				ExtraPinMode extraPinMode = ExtraPinMode.getExtraPinMode(motor, channel);
				if( extraPinMode == ExtraPinMode.ANALOG ){
					slidergroup.addView( extra_value[channel.index] = new Slider("Extra analog", 0, 1024, 0) );
					extra_value[channel.index].grayOut(true);
				}
				else if( extraPinMode == ExtraPinMode.SERVO ){
					slidergroup.addView( extra_value[channel.index] = new Slider("Servo", 0, 255, 100) );
				}
				else if( extraPinMode == ExtraPinMode.SWITCH ){
					channelbox.addView( extra_switch[channel.index] = new JCheckBox("Switch", false) );
					extra_switch[channel.index].setEnabled(false);
				}

				view.addViewToRight(channelbox);

				enable[channel.index].addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						if( enable[channel.index].isSelected() ) motor.setChannelParameter(channel, ChannelParameter.ENABLED, 1);
						else motor.setChannelParameter(channel, ChannelParameter.ENABLED, 0);
					}
				});

			}

			return view;
		}
	}

	private class ErrorElement{
		public MCBMiniBoard motor;
		public EnumMap<Error, Slider>[] error_sliders;

		public ErrorElement(MCBMiniBoard motorboard){
			this.motor = motorboard;
		}

		public Box init(){
			Box[] channelbox = new Box[2];
			error_sliders = new EnumMap[2];
			for(int i=0; i<2; i++) error_sliders[i] = new EnumMap<Error, Slider>(Error.class);
			Box motorbox = new Box("Motorboard "+motor.getId());
			SliderGroup non_channel = new SliderGroup();

			for (Error error : Error.values()) {
				if( !error.channel_specific ){
					Slider err_slider = new Slider(error.toString(), 0, 200, 0);
					err_slider.grayOut(true);
					error_sliders[0].put(error, err_slider);
					non_channel.addView( err_slider );
				}
			}

			for(int i=0; i<2; i++){
				Channel ch = i==0 ? Channel.A : Channel.B;
				channelbox[i] = new Box("Channel "+ch.toString());
				SliderGroup slidergroup = new SliderGroup();
				channelbox[i].addView(slidergroup);

				for (Error error : Error.values()) {
					if( error.channel_specific ){
						Slider err_slider = new Slider(error.toString(), 0, 200, 0);
						err_slider.grayOut(true);
						error_sliders[ch.index].put(error, err_slider);
						slidergroup.addView( err_slider );
					}
				}
			}

			motorbox.addView(non_channel);
			motorbox.addView(channelbox[0]);
			motorbox.addViewToRight(channelbox[1]);

			return motorbox;
		}

		public void update(){
			for (Error error : Error.values()) {
				if( error.channel_specific ){
					for (Channel ch : Channel.values()) {
						error_sliders[ch.index].get(error).setValue( motor.getErrorCount(error, ch) );
					}
				}
				else{
					error_sliders[0].get(error).setValue( motor.getErrorCount(error) );
				}
			}
		}
	}

	private class SetParametersElement{
		public Slider[] pos_p_gain = new Slider[2];
		public Slider[] pos_d_gain = new Slider[2];
		public Slider[] pos_i_gain = new Slider[2];
		public Slider[] pos_downscale = new Slider[2];

		public Slider[] vel_p_gain = new Slider[2];
		public Slider[] vel_d_gain = new Slider[2];
		public Slider[] vel_i_gain = new Slider[2];
		public Slider[] vel_downscale = new Slider[2];
		public Slider[] vel_time_delta = new Slider[2];

		public Slider[] max_vel = new Slider[2];
		public Slider[] max_acc = new Slider[2];


		public RadioButton[] polarity = new RadioButton[2];
		public RadioButton[] feedback = new RadioButton[2];
		public RadioButton[] velocity = new RadioButton[2];
		public RadioButton[] stream = new RadioButton[2];

		public JButton[] buttons = new JButton[2];

		public MCBMiniBoard motor;

		public SetParametersElement(MCBMiniBoard motorboard){
			this.motor = motorboard;
		}

		public Box init(){
			Box motorbox = new Box("Motorboard "+motor.getId());
			Box[] channelbox = new Box[2];

			for(int i=0; i<2; i++){
				Channel ch = i==0 ? Channel.A : Channel.B;

				channelbox[i] = new Box("Channel "+ch.toString());

				Box position_box = new Box("Position");
				channelbox[i].addView(position_box);
				SliderGroup slidergroup = new SliderGroup();
				position_box.addView(slidergroup);
				slidergroup.addView( pos_p_gain[i] = new Slider("P Gain", 0, 600, motor.getChannelParameter(ch, ChannelParameter.POS_P_GAIN)) );
				slidergroup.addView( pos_d_gain[i] = new Slider("D Gain", 0, 600, motor.getChannelParameter(ch, ChannelParameter.POS_D_GAIN)));
				slidergroup.addView( pos_i_gain[i] = new Slider("I Gain", 0, 600, motor.getChannelParameter(ch, ChannelParameter.POS_I_GAIN)));
				slidergroup.addView( pos_downscale[i] = new Slider("Downscale", 0, 10, motor.getChannelParameter(ch, ChannelParameter.POS_DOWNSCALE)));

				Box velocity_box = new Box("Velocity");
				channelbox[i].addView(velocity_box);
				slidergroup = new SliderGroup();
				velocity_box.addView(slidergroup);
				slidergroup.addView( vel_p_gain[i] = new Slider("P Gain", 0, 600, motor.getChannelParameter(ch, ChannelParameter.VEL_P_GAIN)) );
				slidergroup.addView( vel_d_gain[i] = new Slider("D Gain", 0, 600, motor.getChannelParameter(ch, ChannelParameter.VEL_D_GAIN)));
				slidergroup.addView( vel_i_gain[i] = new Slider("I Gain", 0, 600, motor.getChannelParameter(ch, ChannelParameter.VEL_I_GAIN)));
				slidergroup.addView( vel_downscale[i] = new Slider("Downscale", 0, 10, motor.getChannelParameter(ch, ChannelParameter.VEL_DOWNSCALE)));
				slidergroup.addView( vel_time_delta[i] = new Slider("Time Delta", 2, 5, motor.getChannelParameter(ch, ChannelParameter.VEL_TIME_DELTA)));


				slidergroup = new SliderGroup();
				channelbox[i].addView(slidergroup);
				slidergroup.addView( max_vel[i] = new Slider("Maximum Velocity", 0, 100, motor.getChannelParameter(ch, ChannelParameter.MAX_VELOCITY)));
				slidergroup.addView( max_acc[i] = new Slider("Maximum Acceleration", 0, 100, motor.getChannelParameter(ch, ChannelParameter.MAX_ACCELERATION)));

				channelbox[i].addView(polarity[i] = new RadioButton("Polarity", MotorPolarity.values(), MotorPolarity.getForId( motor.getChannelParameter(ch, ChannelParameter.POLARITY))));
				channelbox[i].addView(feedback[i] = new RadioButton("Feedback", FeedbackMode.values(), FeedbackMode.getForId( motor.getChannelParameter(ch, ChannelParameter.FEEDBACK_MODE))));
				channelbox[i].addView(velocity[i] = new RadioButton("Control", ControlMode.values(), ControlMode.getForId( motor.getChannelParameter(ch, ChannelParameter.CONTROL_MODE))));
				channelbox[i].addView(stream[i] = new RadioButton("Stream", StreamMode.values(), StreamMode.getForId( motor.getChannelParameter(ch, ChannelParameter.STREAM_MODE))));

				channelbox[i].addView(buttons[i] = new JButton("Send parameters"));

				motorbox.addViewToRight(channelbox[i]);

				buttons[i].addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent arg0) {
						System.out.println("Sending parameters");
						Channel ch;
						int ind;
						if (arg0.getSource() == buttons[0]){
							ch = Channel.A;
							ind = 0;
						}
						else{
							ch = Channel.B;
							ind = 1;
						}

						motor.setChannelParameter(ch, ChannelParameter.POS_P_GAIN, (int)(pos_p_gain[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.POS_D_GAIN, (int)(pos_d_gain[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.POS_I_GAIN, (int)(pos_i_gain[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.POS_DOWNSCALE, (int)(pos_downscale[ind].evaluate()));

						motor.setChannelParameter(ch, ChannelParameter.VEL_P_GAIN, (int)(vel_p_gain[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.VEL_D_GAIN, (int)(vel_d_gain[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.VEL_I_GAIN, (int)(vel_i_gain[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.VEL_DOWNSCALE, (int)(vel_downscale[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.VEL_TIME_DELTA, (int)(vel_time_delta[ind].evaluate()));

						motor.setChannelParameter(ch, ChannelParameter.MAX_VELOCITY, (int)(max_vel[ind].evaluate()));
						motor.setChannelParameter(ch, ChannelParameter.MAX_ACCELERATION, (int)(max_acc[ind].evaluate()));

						motor.setChannelParameter(ch, ChannelParameter.POLARITY, MotorPolarity.valueOf(polarity[ind].evaluate()).id);
						motor.setChannelParameter(ch, ChannelParameter.FEEDBACK_MODE, FeedbackMode.valueOf(feedback[ind].evaluate()).id);
						motor.setChannelParameter(ch, ChannelParameter.CONTROL_MODE, ControlMode.valueOf(velocity[ind].evaluate()).id);
						motor.setChannelParameter(ch, ChannelParameter.STREAM_MODE, StreamMode.valueOf(stream[ind].evaluate()).id);
					}
				});
			}

			return motorbox;
		}
	}

	public static class RadioButton extends Box{

		private ButtonGroup group;
		private String selected = null;
		private String[] options;

		public RadioButton(String name, final Enum[] options, Enum initial){
			super(name);
			String[] op = new String[options.length];
			for(int i=0; i<op.length; i++){
				op[i] = options[i].name();
			}
			init(op, initial.name());
		}


		public RadioButton(String name, final String[] options, String initial){
			super(name);
			init(options, initial);
		}

		private void init(final String[] options, String initial){
			this.options = options;

			if( initial == null ){
				this.selected = options[0];
			}
			else{
				this.selected = initial;
			}

			group = new ButtonGroup();
			for(int i=0; i<options.length; i++){
				final String en = this.options[i];
				JRadioButton button = new JRadioButton(en);
				if( en.equals(initial) ) button.setSelected(true);

				button.addActionListener(new ActionListener() {
//					//@Override
					public void actionPerformed(ActionEvent arg0) {
						selected = en;
					}
				});
				add(button);
				group.add(button);
			}
		}

		public String evaluate(){
			return selected;
		}
	}

	private static class Slider extends JPanel{

		private JSlider slider;
		private JTextField input;
		private JLabel label;
		private int value;

		public Slider(String name, int low_val, int high_val, int init_val){
			this.value = init_val;
			add(label = new JLabel(name));
			add(slider = new JSlider(low_val, high_val, init_val));
			add(input = new JTextField(""+init_val, 5));

			input.addActionListener(new ActionListener() {
//				//@Override
				public void actionPerformed(ActionEvent arg0) {
					evaluateTextBox();
				}
			});
			input.addFocusListener(new FocusListener() {
//				//@Override
				public void focusLost(FocusEvent arg0) {
					evaluateTextBox();
				}

//				//@Override
				public void focusGained(FocusEvent arg0) {
				}
			});

			slider.addChangeListener(new ChangeListener() {

//				//@Override
				public void stateChanged(ChangeEvent arg0) {
					input.setText(""+slider.getValue());
					value = slider.getValue();
				}
			});
		}

		private void evaluateTextBox(){
			int val = Integer.MAX_VALUE;
			try{
				val = Integer.parseInt( input.getText() );
			}
			catch (NumberFormatException e) { }
			if( val == Integer.MAX_VALUE ){
				input.setText(""+value);
			}
			else{
				val = Math.max(val, slider.getMinimum());
				val = Math.min(val, slider.getMaximum());
				slider.setValue(val);
				value = val;
			}
		}

		public int evaluate(){
			return value;
		}

		public void setValue(int val){
			input.setText(""+val);

			val = Math.max(val, slider.getMinimum());
			val = Math.min(val, slider.getMaximum());
			slider.setValue(val);

			value = val;
		}

		public void grayOut(boolean gray){
			slider.setEnabled(!gray);
			input.setEnabled(!gray);
		}

	}

	private static class Window extends JFrame{

		private JTabbedPane panel;
		private int y = 0;
		private int x = 0;

		public Window(final String name){
			super(name);
			panel = new JTabbedPane();
			add(panel);

			String x_str = prefs.get(name+"_x", null);
			String y_str = prefs.get(name+"_y", null);
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
					prefs.putInt(name+"_x", getX());
					prefs.putInt(name+"_y", getY());
				}
			});
		}

		public void addView(String name, Component comp){
			panel.add( name, comp );
		}

	}

	private static class SliderGroup extends JPanel{

		private int y;

		public SliderGroup(){
			super(new GridBagLayout());
			y = 0;
		}

		public void addView(Slider comp){

			GridBagConstraints c = new GridBagConstraints();
			c.anchor = GridBagConstraints.WEST;
			c.weighty = 1;
			c.gridx = 0;
			c.gridy = y++;
			add( comp.label, c );
			c.gridx++;
			add( comp.slider, c );
			c.gridx++;
			add( comp.input, c );
		}
	}

	public static class Box extends JPanel{

		private int y, x;
		private JPanel panel;

		public Box(){
			super(new FlowLayout());

			panel = new JPanel(new GridBagLayout());
			add(panel, FlowLayout.LEFT);

			y = 0;
			x = 0;
		}

		public Box(String title){
			super(new GridBagLayout());
			TitledBorder border = BorderFactory.createTitledBorder(BorderFactory.createEtchedBorder(EtchedBorder.LOWERED), title);
			setBorder(border);

			panel = new JPanel(new GridBagLayout());
			add(panel, FlowLayout.LEFT);

			y = 0;
			x = 0;
		}

		public void addView(Component comp){
			x = 0;
			GridBagConstraints c = new GridBagConstraints();
			c.anchor = GridBagConstraints.NORTHWEST;
			c.weighty = 1;
			c.gridx = 0;
			c.gridy = y++;

			panel.add( comp, c );
		}

		public void addViewToRight(Component comp){
			GridBagConstraints c = new GridBagConstraints();
			c.anchor = GridBagConstraints.WEST;
			c.weighty = 0;
			c.gridx = ++x;
			c.gridy = Math.max(0, y-1);

			panel.add( comp, c );
		}
	}

	private static String[] getLabels(Enum[] obj){
		String[] ret = new String[obj.length];
		for(int i=0; i<obj.length; i++){
			ret[i] = obj[i].toString();
		}
		return ret;
	}
}
