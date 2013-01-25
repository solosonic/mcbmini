How to build the project:
---------------------
 - cd into the "res" folder
 - to create a jar that doesn't include the java dependencies (better for embedding) then do:
   - "ant dist"
   - an MCBMini.jar should appear in the "dist" folder
 - to create a jar that contains its dependencies and is runnable and clickable, do:
   - "ant app"
   - an MCBMiniApp.jar should appear in the "dist" folder

How to run the MCBMiniGUI:
---------------------
 - type "java -jar MCBMiniApp.jar", you will both be presented with command line options as well as a file selector tool for the xml file to use
 - if the jar is located at the same level as the "lib" folder, then it will find the native libraries required, otherwise their location should be specified through a command line option

How to use the API:
---------------------
 - Look at "mcbmini.examples.Example" for reference