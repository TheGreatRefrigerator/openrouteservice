package heigit.ors.routing.graphhopper.extensions.flagencoders;

import com.graphhopper.reader.OSMWay;
import com.graphhopper.routing.util.CarFlagEncoder;
import com.graphhopper.util.Helper;

/**
 * Defines bit layout for cars. (speed, access, ferries, ...)
 * <p>
 */
public class CarTmcFlagEncoder extends CarFlagEncoder {
	
	private String[] TMC_ROAD_TYPES = new String[] { "motorway", "motorway_link", "trunk", "trunk_link", "primary",
			"primary_link", "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential" };

	/**
	 * Should be only instantied via EncodingManager
	 */
	public CarTmcFlagEncoder() {
		this(5, 5, 0);
	}

    public CarTmcFlagEncoder( String propertiesStr )
    {
		     this((int) parseLong(propertiesStr, "speedBits", 5),
		                parseDouble(propertiesStr, "speedFactor", 5),
		                parseBoolean(propertiesStr, "turnCosts", false) ? 3 : 0);
    }

	public CarTmcFlagEncoder(int speedBits, double speedFactor, int maxTurnCosts) {
		super(speedBits, speedFactor, maxTurnCosts);
		
		defaultSpeedMap.put("unclassified", 10);  
        defaultSpeedMap.put("residential", 10);
	}

	@Override
	public long acceptWay(OSMWay way) {
		String highwayValue = way.getTag("highway");

		if (Helper.isEmpty(highwayValue))
			return 0;

		boolean accept = false;
		for (int i = 0; i < TMC_ROAD_TYPES.length; i++) {
			if (TMC_ROAD_TYPES[i].equalsIgnoreCase(highwayValue)) {
				accept = true;
				break;
			}
		}

		if (!accept)
			return 0;

		return super.acceptWay(way);
	}

	@Override
	public String toString() {
		return "cartmc";
	}
}