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

package mcbmini.serial;

import java.io.File;

/**
 * @author siggi
 * @date Jul 18, 2012
 */
public class MCBMiniNativeLoader {

	private static boolean should_load_native = true;

	public static boolean shouldLookForNative(){
		return should_load_native;
	}

	public static void disableNativeLoading(){
		should_load_native = false;
	}

	protected static void loadNativeLibrary(){
		if( should_load_native ){
			String lib_path = findLibraryLocation();
			if( lib_path != null ){
				System.load(lib_path);
				return;
			}
			System.err.println("MCBMiniNativeLoader: Tried searching for native library in lib folder, didn't find it");
			System.err.println("Attempting regular library loading");

			try{
				System.loadLibrary("rxtxSerial");
			} catch(java.lang.UnsatisfiedLinkError error){
				System.err.println("MCBMiniNativeLoader: Can't load native rxtx libraries");
				System.err.println(error.getMessage());
				System.exit(0);
			}
		}
	}

	public static String findLibraryLocation(){
		String lib_path = System.getProperty("mcbmini.rxtx.library.path");
		String os_name = System.getProperty("os.name").toLowerCase();
		String os_arch = System.getProperty("os.arch").toLowerCase();
		String SEP = System.getProperty("file.separator").toLowerCase();

		if( lib_path == null ){
			String os_subfolder = null;
			String arch_subfolder = null;
			String os_libname = null;

			if( os_arch.contains("32") || os_arch.contains("x86") ){
				arch_subfolder = "32bit";
			}
			else {
				arch_subfolder = "64bit";
			}

			if( os_name.contains("mac") ){
				os_subfolder = "osx";
				os_libname = "librxtxSerial.jnilib";
			}
			else if( os_name.contains("windows") ){
				os_subfolder = "windows";
				os_libname = "rxtxSerial.dll";
			}
			else if( os_name.contains("linux") ){
				os_subfolder = "linux";
				os_libname = "librxtxSerial.so";
			}
			else{
				System.err.println("Can't find native libraries for your OS");
				return null;
			}

			lib_path = System.getProperty("user.dir") + SEP + "lib"+ SEP +os_subfolder + SEP + arch_subfolder+SEP +os_libname;
			if( new File(lib_path).exists() ){
				return lib_path;
			}
			lib_path = System.getProperty("user.dir") + SEP + ".." + SEP + "lib"+ SEP +os_subfolder + SEP + arch_subfolder+SEP +os_libname;
			if( new File(lib_path).exists() ){
				return lib_path;
			}

		}
		return null;
	}
}
