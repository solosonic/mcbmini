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

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import mcbmini.MCBMiniBoard;
import mcbmini.MCBMiniConstants;
import mcbmini.MCBMiniConstants.ChannelParameter;
import mcbmini.MCBMiniConstants.ExtraPinMode;
import mcbmini.MCBMiniConstants.MotorPolarity;
import mcbmini.MCBMiniConstants.ControlMode;
import mcbmini.MCBMiniServer;

import org.jdom.Attribute;
import org.jdom.Document;
import org.jdom.Element;
import org.jdom.JDOMException;
import org.jdom.input.SAXBuilder;


/**
 * @author siggi
 * @date Jul 26, 2011
 */
public class XMLUtils {


	public static class XMLResults{
		public ArrayList<MCBMiniBoard> boards;
		public String port_name;
		public int minimum_firmware_version;
	}

	public static Element loadXMLFile(String path) throws Exception{
		SAXBuilder builder = new SAXBuilder();
		Document doc;

		File in_file = new File(path);

		if(!in_file.exists()){
			throw new Exception("Can't find configuration file: "+path);
		}

		Element root = null;
		try {
			doc = builder.build(in_file);
			root = doc.getRootElement();
		} catch (JDOMException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}



		return root;
	}

	private static Element findMCBRoot(Element root_candidate){
		if( root_candidate.getChild("MiniBoards") != null ) return root_candidate;

		for(Object e : root_candidate.getChildren()){
			if( e instanceof Element ){
				Element root = findMCBRoot( (Element)e );
				if( root != null ) return root;
			}
		}
		return null;
	}

	public static XMLResults parseMCBMiniConfigFile(String path) throws Exception{
		XMLResults res = new XMLResults();

		Element root = findMCBRoot( loadXMLFile(path) );
		if( root == null ){
			throw new Exception("Can't find <Root> tag in configuration file");
		}

		double xml_version = -1;
		Attribute attr_xml_version = root.getAttribute("version");
		if( attr_xml_version != null ){
			xml_version = Double.parseDouble( attr_xml_version.getValue() );
		}
		else{
			xml_version = 1.0; // This is the version number for the modern XML format that just hasn't got the "version" flag yet
		}

		res.port_name = getOptional(root, "port");
		res.boards = new ArrayList<MCBMiniBoard>();

		// See if this configuration file requires a certain minimum firmware version
		String min_firmv_version = getOptional(root, "firmware_version");
		if( min_firmv_version != null ) res.minimum_firmware_version = Integer.parseInt(min_firmv_version);

		if(root.getChild("MiniBoards") == null ) throw new RuntimeException("XML file has no MiniBoards list !");
		List<Element> miniBoards = root.getChild("MiniBoards").getChildren();

		// Here we do a quick test to check if an outdated configuration file is being used (created before versioning numbers)
		if( miniBoards.size() > 0 ){
			Element first_board = miniBoards.get(0);
			if( first_board.getChild("Channels") != null && first_board.getChild("Channels").getChild("Motor") != null &&  first_board.getChild("Channels").getChild("Motor").getChild("pgain") != null ){
				xml_version = 0.1;
			}
		}
		Log.println("MCBMini: Parsing configuration file \""+path+"\" of version: "+xml_version);

		if( xml_version < MCBMiniServer.getMinimumConfigFileVersion() ){
			throw new RuntimeException("MCBMini: Config file is outdated, please upgrade to a more modern version. This version of MCBMiniServer requires a minimum version number of: "+MCBMiniServer.getMinimumConfigFileVersion());
		}

		// Finally we process every board
		for(Element board : miniBoards){
			// Parse board information
			MCBMiniBoard miniBoard = parseMCBMiniBoard(board);

			res.boards.add(miniBoard);
		}

		return res;
	}


	public static MCBMiniBoard parseMCBMiniBoard(Element board){
		if( !board.getName().equals("MiniBoard") ) throw new RuntimeException("XML error: Can't parse parameters for MCBMiniBoard");

		int id = Integer.parseInt(getRequired(board, "id"));

		MCBMiniBoard miniBoard = new MCBMiniBoard( id );

		if(board.getChild("Channels") == null) throw new RuntimeException("XML error: MCBMini board "+id+"  has no \"Channels\"!");
		List<Element> channels = board.getChild("Channels").getChildren();
		boolean[] channels_done = new boolean[2];

		String pid_update_str = getOptional(board, "pid_update_period");
		if( pid_update_str != null ){
			Integer pid_update = Integer.parseInt( pid_update_str );
			miniBoard.setPIDUpdatePeriod(pid_update);
		}

		for(Element motor : channels){
			MCBMiniConstants.Channel channel = getRequiredEnum(motor, "channel", MCBMiniConstants.Channel.values());
			channels_done[channel.index] = true;

			miniBoard.setPositionPGain(channel, Integer.parseInt(getRequired(motor, "pos_p_gain")));
			miniBoard.setPositionDGain(channel, Integer.parseInt(getRequired(motor, "pos_d_gain")));
			miniBoard.setPositionIGain(channel, Integer.parseInt(getRequired(motor, "pos_i_gain")));
			miniBoard.setPositionDownscale(channel, Integer.parseInt(getRequired(motor, "pos_downscale")));
			miniBoard.setVelocityPGain(channel, Integer.parseInt(getRequired(motor, "vel_p_gain")));
			miniBoard.setVelocityDGain(channel, Integer.parseInt(getRequired(motor, "vel_d_gain")));
			miniBoard.setVelocityIGain(channel, Integer.parseInt(getRequired(motor, "vel_i_gain")));
			miniBoard.setVelocityDownscale(channel, Integer.parseInt(getRequired(motor, "vel_downscale")));
			miniBoard.setVelocityTimeDelta(channel, Integer.parseInt(getRequired(motor, "vel_time_delta")));

			miniBoard.setMaxVelocity(channel, Integer.parseInt(getRequired(motor, "max_velocity")));
			miniBoard.setMaxAcceleration(channel, Integer.parseInt(getRequired(motor, "max_acceleration")));
			miniBoard.setSlowEnableConstant(channel, Integer.parseInt(getRequired(motor, "slow_enable_const")));

			miniBoard.setPolarity(channel, getRequiredEnum(motor, "polarity", MCBMiniConstants.MotorPolarity.values()));
			miniBoard.setFeedbackMode(channel, getRequiredEnum(motor, "feedback", MCBMiniConstants.FeedbackMode.values()));
			miniBoard.setControlMode(channel, getRequiredEnum(motor, "control", MCBMiniConstants.ControlMode.values()));
			miniBoard.setStreamMode(channel, getRequiredEnum(motor, "stream", MCBMiniConstants.StreamMode.values()));

			ExtraPinMode extra_pin_mode = getOptionalEnum(motor, "extra_pin", MCBMiniConstants.ExtraPinMode.values());
			if( extra_pin_mode != null ) miniBoard.setExtraPinMode(channel, extra_pin_mode);
		}
		if( !(channels_done[0] && channels_done[1]) ) throw new RuntimeException("XML error: MCBMini board "+id+" doesn't specify both channels");

		return miniBoard;
	}

	public static String getRequired(Element e, String childName){
		Element childElement = e.getChild(childName);
		if(childElement == null ) throw new RuntimeException("XML error: MCBMini-"+e+" missing child "+ childName);
		String value = childElement.getValue();
		if(value == null) throw new RuntimeException("XML error: MCBMini-"+e+"'s child "+childName+" is missing value");
		return value;
	}

	public static String getOptional(Element e, String childName){
		Element childElement = e.getChild(childName);
		if(childElement == null ) return null;
		String value = childElement.getValue();
		return value;
	}

	public static <T extends Enum> T getRequiredEnum(Element e, String childName, T[] enums){
		T optionalEnum = getOptionalEnum(e, childName, enums);
		if( optionalEnum == null ) throw new RuntimeException("XML error: MCBMini-"+e+" missing child "+ childName);
		return optionalEnum;
	}

	public static <T extends Enum> T getOptionalEnum(Element e, String childName, T[] enums){
		String value = getOptional(e, childName);
		if( value == null ) return null;

		for(T en: enums){
			if(en.name().equals(value)){
				return en;
			}
		}
		String enumsAsOneString = "";
		String delim = "";
		for(Enum en: enums){
			enumsAsOneString += delim+en.name();
			delim = ", ";
		}
		throw new IllegalArgumentException("XML error: Error, value "+value+" matches none of enums:"+enumsAsOneString);
	}

}
