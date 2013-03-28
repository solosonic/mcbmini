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

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.TimerTask;

import javax.management.RuntimeErrorException;
import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MaximizeAction;

import java.util.Timer;

import mcbmini.MCBMiniConstants.Channel;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.Command;
import mcbmini.MCBMiniConstants.DataSize;
import mcbmini.MCBMiniConstants.Error;
import mcbmini.MCBMiniConstants.ExtraPinMode;
import mcbmini.MCBMiniConstants.Id;
import mcbmini.MCBMiniSerialManager.ResponseType;
import mcbmini.functions.TargetFunction;
import mcbmini.utils.ByteBufferUtils;
import mcbmini.utils.FramerateMonitor;
import mcbmini.utils.Log;

import org.jdom.Element;


/**
 * @author siggi
 * @date Feb 16, 2011
 */
public class MCBMiniServer{

	private static final double server_version = 1.0;

	private static final double minimum_config_file_version = 1.0;

	public static double getServerVersion(){
		return server_version;
	}

	public static double getMinimumConfigFileVersion(){
		return minimum_config_file_version;
	}

	public static final boolean DEBUG = false;

	protected static boolean SHOULD_RESEND_LAST_KNOWN_TICKS_ON_BOARD_RESET = true;
	protected static boolean SHOULD_RESEND_LAST_ENABLED_VALUE_ON_BOARD_RESET = true;

	static{
		// Check for inconsistency in the options above
		if( SHOULD_RESEND_LAST_ENABLED_VALUE_ON_BOARD_RESET && !SHOULD_RESEND_LAST_ENABLED_VALUE_ON_BOARD_RESET ){
			throw new RuntimeException("MCBMiniMotorServer: This is dangerous, I am being told to re-enable boards on reset but not give them the last known tick values. This could lead to an uncalibrated restart.");
		}
	}

	protected int minimum_firmware_version = 16;

	private static final float DEFAULT_UPDATE_RATE = 50f;

	private static final int BAUD_RATE = 115200;

	public static abstract class MCBMiniResponseHandler{
		public abstract void handleResponse(MCBMiniBoard board, Channel channel, Command command, int value);

		public void handleTimeout(MCBMiniBoard board, Channel channel, Command command){
			Log.println("TIMEOUT while waiting for response from board: "+board.getId()+", channel: "+channel+", command: "+command, true);
		}
	}

	public static abstract class MCBMiniIDResponseHandler{
		public abstract void handleIDResponse(int board_id);

		public void handleTimeout(int board_id){
			Log.println("TIMEOUT while waiting for ID response", true);
		}
	}

	public interface MCBMiniBoardDisabledHandler{
		public void handleBoardDisableEvent(MCBMiniBoard board, Channel ch);
	}

	private class BoardDisabledEvent{
		public MCBMiniBoard board;
		public Channel channel;

		public BoardDisabledEvent(MCBMiniBoard board, Channel ch){
			this.board = board;
			this.channel = ch;
		}
	}

	protected ArrayList<MCBMiniBoard> boards;

	private long last_check_for_timeouts_ms;

	private MCBMiniSerialManager ser_manager;

	private List<Request> incoming_requests;

	private static final Channel[] CHANNELS = Channel.values();

	private ResponseType[] response_types;

	private List<MCBMiniBoardDisabledHandler> board_disable_event_handlers;
	private List<BoardDisabledEvent> board_disable_events;


	private List<Request> responses_to_be_handled;
	private HashMap<RequestWrapper, ArrayList<TimestampedResponseHandler>> response_handlers;
	private ArrayList<TimestampedIDResponseHandler> id_packet_handlers;

	private HashMap<Integer, MCBMiniBoard> board_id_to_board_map;

	private FramerateMonitor internal_upd_fm = new FramerateMonitor(2);
	private FramerateMonitor all_board_upd_fm = new FramerateMonitor(2);

	public enum FaultHandlingPolicy {DO_NOTHING, RE_ENABLE};

	private int board_firmware_response_count;
	private int lowest_reported_firmware_version;
	private boolean board_firmware_has_been_confirmed = false;;

	private FaultHandlingPolicy fault_handling_policy = FaultHandlingPolicy.DO_NOTHING;

	public MCBMiniServer(String port_name, ArrayList<MCBMiniBoard> boards) throws IOException{
		this(port_name, boards, DEFAULT_UPDATE_RATE);
	}

	public MCBMiniServer(String port_name, ArrayList<MCBMiniBoard> boards, float update_rate) throws IOException{
		MCBMiniSerialManager serman = new MCBMiniSerialManager(port_name, BAUD_RATE);

		init(serman, boards, update_rate);
	}

	public MCBMiniServer(MCBMiniSerialManager ser_manager, ArrayList<MCBMiniBoard> boards) throws IOException{
		init(ser_manager, boards, DEFAULT_UPDATE_RATE);
	}

	public MCBMiniServer(MCBMiniSerialManager ser_manager, ArrayList<MCBMiniBoard> boards, float update_rate) throws IOException{
		init(ser_manager, boards, update_rate);
	}

	private void init(MCBMiniSerialManager serial_manager, final ArrayList<MCBMiniBoard> boards, final float update_rate) throws IOException{
		this.boards = boards;

		// Start by checking to see if any of the boards have invalid IDs
		ArrayList<Integer> ids = new ArrayList<Integer>();
		for (MCBMiniBoard board : boards) {
			Integer id = Integer.valueOf(board.getId());
			if( id < 0 || id > 126 ){
				throw new RuntimeException("Invalid MCBMiniBoard ID! needs to be between 0 and 126");
			}
			if( ids.contains(id) ){
				throw new RuntimeException("Invalid MCBMiniBoard ID! two boards found with the id "+id);
			}
			ids.add(id);
		}

		last_check_for_timeouts_ms = -1;

		this.ser_manager = serial_manager;

		if(ser_manager == null){
			Log.println("Can't instantiate serial manager for mcbminiserver");
			System.exit(0);
		}

		incoming_requests = Collections.synchronizedList( new ArrayList<MCBMiniServer.Request>() );
		responses_to_be_handled = Collections.synchronizedList( new ArrayList<MCBMiniServer.Request>() );
		response_handlers = new HashMap<RequestWrapper, ArrayList<TimestampedResponseHandler>>();
		id_packet_handlers = new ArrayList<TimestampedIDResponseHandler>();

		board_disable_event_handlers = new ArrayList<MCBMiniBoardDisabledHandler>();
		board_disable_events = new ArrayList<MCBMiniServer.BoardDisabledEvent>();

		// Create maps for speed
		board_id_to_board_map = new HashMap<Integer, MCBMiniBoard>();
		for (MCBMiniBoard board : boards) {
			board_id_to_board_map.put(Integer.valueOf(board.getId()), board);
		}

		/*
		 * This will set the sleep granularity to 1ms for some reason
		 * (source: http://stackoverflow.com/questions/824110/accurate-sleep-for-java-on-windows)
		 */
		new Thread(new Runnable() {
			@Override
			public void run() {
				try {
					Thread.sleep(Long.MAX_VALUE);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}).start();

		/*
		 * This is our update thread that guarantees our motor framerate
		 */
		final long target_nanosecs_wait = (long)(1.0E9 / (double)update_rate);
		Thread t2 = new Thread(new Runnable() {

			double i_error = 0;
			long last_time = -1;
			long actual_nanosecs_wait;

			double sum = 0;

			@Override
			public void run() {

				while(true){
					long start_time = System.nanoTime();
					long last_duration;
					if( last_time == -1 ) last_duration = target_nanosecs_wait;
					else last_duration = start_time - last_time;

					// A little PI controller
					double error = target_nanosecs_wait-last_duration;
					double control = 0.1 * error + 0.5 * i_error;
					i_error += error;

					// Here we adjust the actual wait for this loop to account for the errors in the last loop
					actual_nanosecs_wait = target_nanosecs_wait + (long)control;
					actual_nanosecs_wait = Math.min((long)(1.8*target_nanosecs_wait), actual_nanosecs_wait);
					actual_nanosecs_wait = Math.max((long)(0.2*target_nanosecs_wait), actual_nanosecs_wait);

					// Here we do our magic
					internalUpdate();

					try {
						// Sleep for the requested amount of time (minus a millisecond or two)
						Thread.sleep( Math.max(0, (actual_nanosecs_wait - (System.nanoTime()-start_time)) / 1000000 - 2 ));

						// Now yield for the last half millisecond
						while( System.nanoTime()-start_time < actual_nanosecs_wait   ){
							Thread.yield();
						}
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}

					last_time = start_time;
				}
			}
		});
		t2.setPriority(Thread.MAX_PRIORITY);
		t2.start();

		/*
		 * Start by sending all the boards an empty message, this could clear out the RX buffers (and flush bad checksums)
		 */
		for (MCBMiniBoard board : boards) {
			sendRequest(board, Channel.A, Command.EMPTY_RESPONSE, 0);
		}

		Runtime.getRuntime().addShutdownHook(new Thread(new Runnable() {
			//			//@Override
			public void run() {
				for (MCBMiniBoard board : boards) {
					board.setChannelAParameter(ChannelParameter.ENABLED, 0);
					board.setChannelBParameter(ChannelParameter.ENABLED, 0);
				}
			}
		}));

		/*
		 * Here we check to see if the firmware of all connected boards is good enough
		 */
		board_firmware_response_count = 0;
		lowest_reported_firmware_version = Integer.MAX_VALUE;

		for (MCBMiniBoard board : boards) {
			sendRequestForResponse(board, Channel.A, Command.FIRMWARE_VERSION, new FirmwareCheckingResponseHandler());
		}
		String str = "Waiting for all boards to report their firmware, IDs: ";
		for (int i=0; i<boards.size()-1; i++) str += boards.get(i).getId()+", ";
		Log.println(str+boards.get(boards.size()-1).getId()+":");
	}

	protected ResponseType[] createResponseTypes(){
		if( lowest_reported_firmware_version <= 16 ){
			return new ResponseType[]{
					ResponseType.ACTUAL_TICK, ResponseType.ACTUAL_TICK,
					ResponseType.ACTUAL_TICK, ResponseType.ACTUAL_TICK,
					ResponseType.ACTUAL_TICK, ResponseType.ACTUAL_TICK,
					ResponseType.ACTUAL_TICK, ResponseType.ACTUAL_TICK,
					ResponseType.ACTUAL_TICK, ResponseType.ACTUAL_TICK,
					ResponseType.MOTOR_CURRENT, ResponseType.MOTOR_CURRENT,
			};
		}
		else{
			return new ResponseType[]{
					ResponseType.ACTUAL_TICK_TWO,
					ResponseType.ACTUAL_TICK_TWO,
					ResponseType.ACTUAL_TICK_TWO,
					ResponseType.ACTUAL_TICK_TWO,
					ResponseType.ACTUAL_TICK_TWO,
					ResponseType.MOTOR_CURRENT_TWO,
			};
		}
	}

	public int getMinimumFirmwareVersion(){
		return this.minimum_firmware_version;
	}

	public FaultHandlingPolicy getFaultHandlingPolicy(){
		return fault_handling_policy;
	}

	public void setFaultHandlingPolicy(FaultHandlingPolicy policy){
		fault_handling_policy = policy;
	}

	public void setMinimumFirmwareVersion(int firmware_version){
		if( firmware_version < this.minimum_firmware_version ) throw new RuntimeException("Can't set the minimum firmware version to be lower than the one already set: "+minimum_firmware_version);
		this.minimum_firmware_version = firmware_version;
	}

	public void addBoardDisabledEventHandler(MCBMiniBoardDisabledHandler handler){
		synchronized (board_disable_event_handlers) {
			if( !board_disable_event_handlers.contains(handler) ){
				board_disable_event_handlers.add(handler);
			}
		}
	}

	public boolean removeBoardDisabledEventHandler(MCBMiniBoardDisabledHandler handler){
		boolean ret;
		synchronized (board_disable_event_handlers) {
			ret = board_disable_event_handlers.remove(handler);
		}
		return ret;
	}


	public int getNumberOfBadChecksums(){
		if( ser_manager == null ) return 0;
		return ser_manager.getNumberOfBadChecksums();
	}

	public void waitForServerInitialization(){
		int count = 0;
		while( !board_firmware_has_been_confirmed ){
			update();
			count++;

			if( count > 100 ) throw new RuntimeException("Timeout while waiting for server to initialize, check to see if you have the right boards id's in the config file and actually connected to the bus");

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	/**
	 * This method gets called at the update rate of the controllers from within the update thread
	 */
	private int internal_update_counter = 0;
	private int responses_within_update_counter = 0;
	private void internalUpdate(){
		responses_within_update_counter = 0;
		internal_update_counter++;
		internal_upd_fm.update();

		/*
		 * Handle all commands received from the boards
		 */
		for (ByteBuffer buffer : ser_manager.handleBufferedSerialData()) {
			handleCommandInBuffer( buffer );
		}

		if( board_firmware_has_been_confirmed ){
			/*
			 * Check packet response flags
			 */
			long cur_time = System.currentTimeMillis();
			if( last_check_for_timeouts_ms == -1 ) last_check_for_timeouts_ms = cur_time;
			if( cur_time - last_check_for_timeouts_ms > 1000 ){
				for (MCBMiniBoard board : boards) {
					if( cur_time - board.last_received_message_ms  > 500 ){
						Log.println("No response from board ID: "+board.getId());
						board.increaseErrorCount(Error.NO_RESPONSE);
					}
				}
				last_check_for_timeouts_ms = cur_time;
			}

			/*
			 * See if we need to update parameters to any board
			 */
			for (MCBMiniBoard board : boards) {
				for (Channel channel : Channel.values()) {
					HashMap<Command, Integer> dirtyParameters = board.getDirtyParameters(channel);
					if( dirtyParameters != null ){
						ser_manager.writeParameterPacket(board, channel, dirtyParameters);

						// Here we stop the current loop as we already have a big buffer to push out
						ser_manager.sendTxBuffer();
						return;
					}
				}
			}

			/*
			 * Then we send updated positions and get positions and currents back
			 */
			// Alternate feedback from the two channels of all boards
			Channel response_channel = CHANNELS[ internal_update_counter % 2 ];

			// Every Xth time, we get electric current information instead of position feedback
			ResponseType response_type = response_types[ internal_update_counter % response_types.length ];

			/*
			 * For older firmware we just stream positions all the time
			 */
			for (MCBMiniBoard board : boards) {
				/*
				 * Here we check to see if either channel has an active target function, if so then we apply its value
				 */
				for (Channel channel : Channel.values()) {
					board.applyTargetFunction(channel);
				}

				/*
				 * Now we apply the actual positions
				 */
				Integer target_A, target_B;
				if( minimum_firmware_version < 16 ){
					target_A = board.getTargetTick(Channel.A);
					target_B = board.getTargetTick(Channel.B);
				}
				// Otherwise we send a special value when the target position hasn't changed
				else{
					target_A = board.getFreshTargetTick(Channel.A);
					target_B = board.getFreshTargetTick(Channel.B);
					if( target_A == null ) target_A = Integer.MAX_VALUE;
					if( target_B == null ) target_B = Integer.MAX_VALUE;
				}

				ser_manager.writeSpecializedPacket(board, response_type, response_channel, target_A, target_B);

				/*
				 * Here we handle the Extra pin functionality
				 */
				ExtraPinMode extraPinMode = board.getExtraPinMode(response_channel);
				if( extraPinMode == ExtraPinMode.ANALOG ){
					ser_manager.writeGenericPacket(board, response_channel, Command.EXTRA_PIN_VALUE, true, 0);
				}
				else if( extraPinMode == ExtraPinMode.SERVO ){
					ser_manager.writeGenericPacket(board, response_channel, Command.EXTRA_PIN_VALUE, false, board.getExtraPinValue(response_channel));
				}
			}
		}
		// Otherwise we just send empty packets
		else{
			for (MCBMiniBoard board : boards) {
				ser_manager.writeGenericPacket(board, CHANNELS[ internal_update_counter % 2 ], Command.EMPTY_RESPONSE, false, 0);
			}
		}

		/*
		 * Now we see if we need to forward any requests
		 */
		synchronized (incoming_requests) {
			Iterator<Request> i = incoming_requests.iterator();
			while(i.hasNext()){
				Request r = i.next();
				ser_manager.writeGenericPacket(r.board, r.channel, r.command, r.should_get_response, r.value);
				i.remove();
			}
		}

		// Send out whole buffer over serial
		ser_manager.sendTxBuffer();
	}

	public List<MCBMiniBoard> getBoards(){
		return Collections.unmodifiableList(boards);
	}


	public float[] getUpdateRates(float[] in){
		if( in == null ) in = new float[2];

		in[0] = internal_upd_fm.getFPS();
		in[1] = all_board_upd_fm.getFPS();
		return in;
	}

	/**
	 * This method posts a request down to a motorboard and when a reply arrives, it calls all registered broadcast handlers with the response
	 */
	public void broadcastRequestForIDResponse(MCBMiniIDResponseHandler resp_handler){
		Request resp = new Request(null, Channel.A, Command.ID, true, 0);

		// Register handler
		id_packet_handlers.add( new TimestampedIDResponseHandler(resp_handler, resp, System.currentTimeMillis()) );

		synchronized ( incoming_requests ) {
			incoming_requests.add(resp);
		}
	}

	/**
	 * This method posts a request down to a motorboard and when a reply arrives, it calls a handler with the response
	 */
	public void sendRequestForResponse(MCBMiniBoard board, Channel channel, Command command, MCBMiniResponseHandler resp_handler){
		if( board == null ){
			throw new RuntimeException("Can't send a request to board null, use broadcast");
		}

		Request resp = new Request(board, channel, command, true, 0);

		// Register handler
		if( resp_handler != null ){
			RequestWrapper req_wrap = new RequestWrapper(resp);
			ArrayList<TimestampedResponseHandler> handler_list = response_handlers.get(req_wrap);
			if( handler_list == null ){
				handler_list = new ArrayList<MCBMiniServer.TimestampedResponseHandler>();
				response_handlers.put(req_wrap, handler_list);
			}
			handler_list.add( new TimestampedResponseHandler(resp_handler, resp, System.currentTimeMillis()) );
		}
		synchronized ( incoming_requests ) {
			incoming_requests.add(resp);
		}
	}

	/**
	 * This method posts a request down to all motorboards and doesn't anticipate an answer
	 */
	public void broadcastRequest(Channel channel, Command command, int value){

		Request resp = new Request(null, channel, command, false, value);
		synchronized ( incoming_requests ) {
			incoming_requests.add(resp);
		}
	}


	/**
	 * This method posts a request down to a motorboard and doesn't anticipate an answer
	 */
	public void sendRequest(MCBMiniBoard board, Channel channel, Command command, int value){
		if( board == null ){
			throw new RuntimeException("Can't send a request to board null, use broadcast");
		}

		Request resp = new Request(board, channel, command, false, value);
		synchronized ( incoming_requests ) {
			incoming_requests.add(resp);
		}
	}


	/**
	 * This function should be called from the external thread (maybe GUI thread)
	 * It makes sure that the message handling is all thread safe
	 */
	private long update_counter = 0;
	public void update(){
		update_counter++;

		/*
		 * Take all messages to be handled and pass them to all the handlers
		 */
		synchronized (responses_to_be_handled) {
			Iterator<Request> i = responses_to_be_handled.iterator();
			while(i.hasNext()){
				Request req = i.next();
				boolean handled = false;

				// If this is an ID response then call the ID handlers
				if( req.command == Command.ID ){
					ArrayList<TimestampedIDResponseHandler> to_use = new ArrayList<MCBMiniServer.TimestampedIDResponseHandler>(id_packet_handlers);
					id_packet_handlers.clear();
					for (TimestampedIDResponseHandler ts_handler : to_use) {
						ts_handler.handler.handleIDResponse(req.return_value);
						handled = true;
					}
				}

				// Otherwise just send it to the designated handlers
				else{
					ArrayList<TimestampedResponseHandler> handlers = response_handlers.get( new RequestWrapper(req) );
					if( handlers != null ){
						ArrayList<TimestampedResponseHandler> handlers_use = new ArrayList<MCBMiniServer.TimestampedResponseHandler>(handlers);
						handlers.clear();
						for (TimestampedResponseHandler ts_handler : handlers_use) {
							ts_handler.handler.handleResponse(req.board, req.channel, req.command, req.return_value);
							handled = true;
						}
					}
				}

				//				// If nobody handled this message then we have a problem because nobody registered for it
				//				if( !handled ){
				//					Log.println("ERROR: received response from board "+req.board.getId()+" but no handler was subscribed to it ("+req.command+")");
				//				}
				i.remove();
			}
		}

		/*
		 * Every now and then we remove response handlers because of timeouts
		 */
		if( update_counter % 50 ==0 ){
			long now = System.currentTimeMillis();

			Iterator<TimestampedIDResponseHandler> ii = id_packet_handlers.iterator();
			List<TimestampedIDResponseHandler> timedout_id = new ArrayList<MCBMiniServer.TimestampedIDResponseHandler>();
			while(ii.hasNext()){
				TimestampedIDResponseHandler ts_handler = ii.next();
				if( now - ts_handler.timestamp > 1000 ){
					timedout_id.add( ts_handler );
					ii.remove();
				}
			}
			for (TimestampedIDResponseHandler ts_handler : timedout_id) {
				ts_handler.handler.handleTimeout(ts_handler.original_request.board==null?Id.BROADCAST.getIDInt():ts_handler.original_request.board.getId());
			}

			List<TimestampedResponseHandler> timedout = new ArrayList<MCBMiniServer.TimestampedResponseHandler>();
			for (ArrayList<TimestampedResponseHandler> list : response_handlers.values()) {
				Iterator<TimestampedResponseHandler> iii = list.iterator();
				while(iii.hasNext()){
					TimestampedResponseHandler ts_handler = iii.next();
					if( now - ts_handler.timestamp > 1000 ){
						timedout.add( ts_handler );
						iii.remove();
					}
				}
				for (TimestampedResponseHandler ts_handler : timedout) {
					ts_handler.handler.handleTimeout(ts_handler.original_request.board, ts_handler.original_request.channel, ts_handler.original_request.command);
				}
				timedout.clear();
			}
		}

		/*
		 * Notify handlers that boards were disabled
		 */
		synchronized (board_disable_event_handlers) {
			for (BoardDisabledEvent ev : board_disable_events) {
				for (MCBMiniBoardDisabledHandler handler : board_disable_event_handlers) {
					handler.handleBoardDisableEvent(ev.board, ev.channel);
				}
			}
			board_disable_events.clear();
		}
	}

	/**
	 * Internal method for handling a single command that is placed in the front of the buffer
	 * @param bb
	 */
	private void handleCommandInBuffer(ByteBuffer bb){
		int handler_value = Integer.MAX_VALUE;

		if(bb.limit()-bb.position()<1){
			Log.println("Weird, got small bb. pos:"+bb.position()+" lim:"+bb.limit()+" cap:"+bb.capacity());
		}
		int id = ByteBufferUtils.getFromBack(bb) & 0xff;
		Channel ch = ((id >> 7) & 0x01)==0 ? Channel.A : Channel.B;		// Here we pick out the channel bit
		id &= 0x7F;														// Mask out the channel bit

		int cmd_byte = ByteBufferUtils.getFromBack(bb) & 0xff;
		boolean has_message = ((cmd_byte >> 7) & 0x01)==1;					// Here we check to see if this slave has a message for us
		cmd_byte &= 0x7F;													// Mask out the message bit

		Command command = Command.getForCmdId( cmd_byte );
		MCBMiniBoard board = board_id_to_board_map.get( id );

		if( board == null && command != Command.ID ){
			Log.println("Received response "+command+" from board id "+id+" that is not in our list, ignoring");
			return;
		}

		if( board != null ) board.last_received_message_ms = System.currentTimeMillis();

		if( DEBUG ) Log.println("Received from id "+id+" command: "+command+" channel "+ ch);

		// If this board has a message to give us then we request to get it
		if( has_message && board != null){
			sendRequestForResponse(board, ch, Command.REQUEST_MESSAGE, null);
		}

		// If this is the first response since last update
		if( responses_within_update_counter == 0 ){
			all_board_upd_fm.update();
		}
		responses_within_update_counter++;

		// If this is a response to our target pos special then just put current data into the motor objects
		if( command == Command.TWO_TARGET_TICK_ACTUAL || command == Command.TWO_TARGET_TICK_VELOCITY ){
			board.setChannelParameter(ch, ChannelParameter.ACTUAL_TICK,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.TWO_TARGET_TICK_TWO_ACTUAL || command == Command.TWO_TARGET_TICK_TWO_VELOCITY ){
			board.setChannelParameter(Channel.A, ChannelParameter.ACTUAL_TICK,  ByteBufferUtils.getIntFromBack(bb) );
			board.setChannelParameter(Channel.B, ChannelParameter.ACTUAL_TICK,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.TWO_TARGET_TICK_TWO_ENCODER ){
			board.setChannelParameter(Channel.A, ChannelParameter.ACTUAL_ENCODER,  ByteBufferUtils.getIntFromBack(bb) );
			board.setChannelParameter(Channel.B, ChannelParameter.ACTUAL_ENCODER,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.TWO_TARGET_TICK_TWO_POT ){
			board.setChannelParameter(Channel.A, ChannelParameter.ACTUAL_POT,  ByteBufferUtils.getIntFromBack(bb) );
			board.setChannelParameter(Channel.B, ChannelParameter.ACTUAL_POT,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.TWO_TARGET_TICK_TWO_MOTOR_CURRENT ){
			board.setChannelParameter(Channel.A, ChannelParameter.MOTOR_CURRENT,  ByteBufferUtils.getIntFromBack(bb) );
			board.setChannelParameter(Channel.B, ChannelParameter.MOTOR_CURRENT,  ByteBufferUtils.getIntFromBack(bb) );
		}
		// If this is a response to our target pos special then just put current data into the motor objects
		else if( command == Command.TWO_TARGET_TICK_MOTOR_CURRENT ){
			board.setChannelParameter(ch, ChannelParameter.MOTOR_CURRENT,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.TWO_TARGET_TICK_ENCODER ){
			board.setChannelParameter(ch, ChannelParameter.ACTUAL_ENCODER,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.TWO_TARGET_TICK_POT ){
			board.setChannelParameter(ch, ChannelParameter.ACTUAL_POT,  ByteBufferUtils.getIntFromBack(bb) );
		}
		else if( command == Command.ACTUAL_TICK ){
			handler_value = ByteBufferUtils.getIntFromBack(bb);
			board.setChannelParameter(ch, ChannelParameter.ACTUAL_TICK, handler_value );
		}
		// If it is a the value of our extra pin (switch or analog)
		else if( command == Command.EXTRA_PIN_VALUE ){
			handler_value = ByteBufferUtils.getIntFromBack(bb);
			board.setChannelParameter(ch, ChannelParameter.EXTRA_PIN_VALUE, handler_value );
		}
		// If it is a simple empty response to let us know that the board is active, do nothing
		else if( command == Command.DEBUG ){
			int debug_val = ByteBufferUtils.getIntFromBack(bb);
			Log.println("Debug message from board "+id+" channel "+ch+" :"+debug_val);
		}
		// If it is a simple empty response to let us know that the board is active, do nothing
		else if( command == Command.EMPTY_RESPONSE ){
			;
		}
		else if( command == null ){
			Log.println("Received unknown command with CMD byte: "+cmd_byte);
		}
		/*
		 * Here we handle all kinds of errors from the boards
		 */
		else if( command == Command.ERROR ){
			// This is an error condition that occurs sometimes (especially on dragonbot) and it usually means that the robot is trying to notify fault but has problems
			int error_code;
			if( bb.remaining() == 0 ){
				Log.println("Error code missing, assuming that it was a fault message", true);
				error_code = Error.FAULT.id;
			}
			else{
				error_code = ByteBufferUtils.getFromBack(bb) & 0xff;
			}

			Error error = Error.getForId( error_code );
			Log.println("Error/Warning from board: "+board.getId()+" : "+error+" ", true);

			if( error == null ){
				Log.println("\nError message not recognized", true);
			}
			else{
				if( error == Error.BAD_CMD_RCV ){
					int bad_command = ByteBufferUtils.getFromBack(bb) & 0xff;
					Log.println(" => "+bad_command + " : " + Command.getForCmdId(bad_command), true);
				}
				else if( error == Error.FAULT ){
					switch( fault_handling_policy ){
					case DO_NOTHING:
						// Notify about the disable
						synchronized (board_disable_event_handlers) {
							board_disable_events.add( new BoardDisabledEvent(board, ch) );
						}
						break;
					case RE_ENABLE:
						if( board.getEnabled(Channel.A) ){
							sendRequest(board, Channel.A, Command.ENABLE, 1);
							Log.println("\nRe-enabling channel A", true);
						}
						if( board.getEnabled(Channel.B) ){
							sendRequest(board, Channel.B, Command.ENABLE, 1);
							Log.println("\nRe-enabling channel B", true);
						}
						break;
					default:
						throw new RuntimeException("Don't have more cases for FaultHandlingPolicy");
					}
				}
				else if( error == Error.PARAM_DUR_EN ){
					board.setChannelParameter(ch, ChannelParameter.ENABLED, 0);

					Log.println("\nParameters set while bridge is enabled, bridge was disabled as a result", true);

					// Notify about the disable
					synchronized (board_disable_event_handlers) {
						board_disable_events.add( new BoardDisabledEvent(board, ch) );
					}
				}
				else if( error == Error.TIMEOUT_DISABLE ){
					board.setChannelAParameter(ChannelParameter.ENABLED, 0);
					board.setChannelBParameter(ChannelParameter.ENABLED, 0);

					// Notify about the disable
					synchronized (board_disable_event_handlers) {
						board_disable_events.add( new BoardDisabledEvent(board, Channel.A) );
						board_disable_events.add( new BoardDisabledEvent(board, Channel.B) );
					}
				}

				// This happens when the motorboard hears the first packet ever, lets us know if it gets reset
				else if( error == Error.UNINITIALIZED ){
					Log.println("\nBoard: "+board.getId()+", channel "+ch+" says it is uninitialized", true);
					Log.println("\tServer responding by sending/resending parameters", true);
					if( SHOULD_RESEND_LAST_ENABLED_VALUE_ON_BOARD_RESET ){
						Log.println("\tAlso sending last known enabled value for channel "+ch+" : "+board.getChannelParameter(ch, ChannelParameter.ENABLED));
					}
					else{
						Log.println("\tSetting channel to DISABLED", true);
						// Notify about the disable
						synchronized (board_disable_event_handlers) {
							if( board.getChannelParameter(ch, ChannelParameter.ENABLED) == 1 ) board_disable_events.add( new BoardDisabledEvent(board, ch) );
						}
						board.setChannelParameter(ch, ChannelParameter.ENABLED, 0);
					}

					if( SHOULD_RESEND_LAST_KNOWN_TICKS_ON_BOARD_RESET ){
						if( board.getChannelParameter(ch, ChannelParameter.FEEDBACK_MODE) == 0){
							int tick = board.getChannelParameter(ch, ChannelParameter.ACTUAL_TICK);
							Log.println("\tAlso sending offset of last known encoder value for channel "+ch+" : "+tick, true);
							sendRequest(board, ch, Command.ACTUAL_ENCODER_OFFSET, tick);
						}
					}
					board.clearParametersInUse(ch);
				}
				else if( error == Error.MSG_BUF_OVF ){
					Log.println("Assuming a fault message was in msg buffer queue", true);
					// This assumes that the message that should have been sent to us was a fault message
					if( fault_handling_policy == FaultHandlingPolicy.RE_ENABLE ){
						if( board.getEnabled(Channel.A) ){
							sendRequest(board, Channel.A, Command.ENABLE, 1);
							Log.println("\nRe-enabling channel A", true);
						}
						if( board.getEnabled(Channel.B) ){
							sendRequest(board, Channel.B, Command.ENABLE, 1);
							Log.println("\nRe-enabling channel B", true);
						}
					}
				}

				if( error.channel_specific ){
					board.increaseErrorCount(error, ch);
				}
				else{
					board.increaseErrorCount(error);
				}
			}
		}
		/*
		 * If we don't know what to do with the message then we just pass it to a handler (someone might have requested it)
		 */
		else{
			// Read the appropriate amount of bytes from the buffer and put it in its place
			if( command.datasize == DataSize.U08 && bb.remaining() > 0 ) 		handler_value = ByteBufferUtils.getFromBack(bb) & 0xff;
			else if( command.datasize == DataSize.S32 && bb.remaining() > 3 ) 	handler_value = ByteBufferUtils.getIntFromBack(bb);
			else{
				Log.println("Received response with command that doesn't have data size specified", true);
			}
		}

		/*
		 * If we should pass the value to handlers
		 */
		if( handler_value != Integer.MAX_VALUE ){
			synchronized (responses_to_be_handled) {
				Request resp =  new Request( board, ch, command, false, 0);
				resp.return_value = handler_value;
				responses_to_be_handled.add(resp);
			}
		}
	}

	private class TimestampedIDResponseHandler{
		public MCBMiniIDResponseHandler handler;
		long timestamp;
		Request original_request;

		public TimestampedIDResponseHandler(MCBMiniIDResponseHandler handler, Request original_request, long timestamp){
			this.handler = handler;
			this.timestamp = timestamp;
			this.original_request = original_request;
		}
	}

	private class TimestampedResponseHandler{
		public MCBMiniResponseHandler handler;
		long timestamp;
		Request original_request;

		public TimestampedResponseHandler(MCBMiniResponseHandler handler, Request original_request, long timestamp){
			this.handler = handler;
			this.timestamp = timestamp;
			this.original_request = original_request;
		}
	}

	private class RequestWrapper{
		private int hash_value;

		public RequestWrapper(Request req){
			this(req.command, req.board, req.channel);
		}

		public RequestWrapper(Command cmd, MCBMiniBoard board, Channel channel){
			hash_value = board.getId() + 100*cmd.hashCode() + 1000* channel.hashCode();
		}

		//@Override
		public boolean equals(Object o) {
			if( o instanceof RequestWrapper ){
				return ((RequestWrapper)o).hash_value == hash_value;
			}
			return false;
		}

		//@Override
		public int hashCode() {
			return hash_value;
		}
	}

	private class Request{
		public Command command;
		public MCBMiniBoard board;
		public Channel channel;
		boolean should_get_response;
		public int value;
		public int return_value;

		public Request(MCBMiniBoard board, Channel channel, Command command, boolean should_get_response, int value){
			this.board = board;
			this.channel = channel;
			this.command = command;
			this.should_get_response = should_get_response;
			this.value = value;
		}
	}

	private class FirmwareCheckingResponseHandler extends MCBMiniResponseHandler{
		@Override
		public void handleResponse(MCBMiniBoard board, Channel channel, Command command, int value) {
			board_firmware_response_count++;
			lowest_reported_firmware_version = Math.min(lowest_reported_firmware_version, value);
			Log.println("Motorboard "+board.getId()+" reports firmware version: "+value);
			if( value < minimum_firmware_version ){
				Log.println("This version of the server can only talk to boards of firmware version "+minimum_firmware_version+" and higher. Board "+board.getId()+" reports "+value, true);
				new RuntimeException().printStackTrace();
				System.exit(0);
			}

			if( board_firmware_response_count == boards.size() ){
				Log.println("All boards have reported their firmware, lowest firmware version: "+lowest_reported_firmware_version);
				response_types = createResponseTypes();
				board_firmware_has_been_confirmed = true;
			}
		}

		@Override
		public void handleTimeout(MCBMiniBoard board, Channel channel, Command command) {
			sendRequestForResponse(board, Channel.A, Command.FIRMWARE_VERSION, new FirmwareCheckingResponseHandler());
		}
	}
}
