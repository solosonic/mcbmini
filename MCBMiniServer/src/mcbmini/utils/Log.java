package mcbmini.utils;

import java.util.Calendar;
import java.util.Date;

/**
 * @author siggi
 * @date Mar 22, 2013
 */
public class Log {
	public static void println(String str){
		println(str, false);
	}

	public static void println(String str, boolean error){
		if( error ) System.err.println(formatString(str));
		else System.out.println(formatString(str));
	}

	protected static String formatString(String str){
		Calendar now = Calendar.getInstance();
		StringBuilder sb = new StringBuilder();
		sb.append(now.get(Calendar.HOUR_OF_DAY));
		sb.append(":");
		sb.append(now.get(Calendar.MINUTE));
		sb.append(":");
		sb.append(now.get(Calendar.SECOND));
		sb.append("-MCBMini: ");
		sb.append(str);
		return sb.toString();
	}
}
