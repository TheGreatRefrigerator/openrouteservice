/*|----------------------------------------------------------------------------------------------
 *|														Heidelberg University
 *|	  _____ _____  _____      _                     	Department of Geography		
 *|	 / ____|_   _|/ ____|    (_)                    	Chair of GIScience
 *|	| |  __  | | | (___   ___ _  ___ _ __   ___ ___ 	(C) 2014
 *|	| | |_ | | |  \___ \ / __| |/ _ \ '_ \ / __/ _ \	
 *|	| |__| |_| |_ ____) | (__| |  __/ | | | (_|  __/	Berliner Strasse 48								
 *|	 \_____|_____|_____/ \___|_|\___|_| |_|\___\___|	D-69120 Heidelberg, Germany	
 *|	        	                                       	http://www.giscience.uni-hd.de
 *|								
 *|----------------------------------------------------------------------------------------------*/

// Authors: M. Rylov

package heigit.ors.routing.traffic.providers;

import java.util.Properties;

public class TrafficInfoDataSourceFactory {
  public static TrafficInfoDataSource create(Properties props) throws Exception
  {
	  TrafficInfoDataSource result = null;
	  
	  String type = props.getProperty("type");
	  
	  if (type.equals("file"))
		  result = new FileDataSource();
	  else if (type.equals("ftp"))
		  result = new FtpDataSource();
	  
	  if (result != null)
	  {
		  result.Initialize(props);
		  return result;
	  }
	  
	  throw new Exception("Unable to detect type of TrafficDataSource");
  }
}
