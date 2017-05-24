package heigit.ors.routing.graphhopper.extensions.graphbuilders;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import com.graphhopper.GraphHopper;
import com.graphhopper.coll.LongIntMap;
import com.graphhopper.reader.OSMReader;
import com.graphhopper.reader.OSMWay;
import com.graphhopper.routing.Dijkstra;
import com.graphhopper.routing.Path;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FastestWeighting;
import com.graphhopper.routing.util.FootFlagEncoder;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.util.Weighting;
import com.graphhopper.storage.GraphExtension;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.storage.RAMDirectory;
import com.graphhopper.util.DistanceCalc;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.util.Helper;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.LinearRing;
import com.vividsolutions.jts.geom.Polygon;

import gnu.trove.list.TIntList;
import gnu.trove.list.TLongList;
import gnu.trove.list.array.TLongArrayList;

public class InFieldGraphBuilder extends AbstractGraphBuilder {

	private GeometryFactory geometryFactory = new GeometryFactory();
	private Map<Integer, Integer> intId2idx = new HashMap<Integer, Integer>(); 
	private Map<Integer, Integer> idx2intId =  new HashMap<Integer, Integer>(); 
	private Map<Integer, Long> intId2osmId = new HashMap<Integer, Long>();
	private ArrayList<Integer> internalTowerNodeIds = new ArrayList<Integer>(); 
	private Coordinate[] _coordinates;
	private Set<ArrayList<Integer>> _edges = new HashSet<ArrayList<Integer>>();
	private ArrayList<Integer> tmpEdge = new ArrayList<Integer>();   
	private FootFlagEncoder footEncoder; 
	private List<Weighting> weightings;
	private EncodingManager encodingManager;

	@SuppressWarnings("resource")
	@Override
	public void init(GraphHopper graphhopper) throws Exception {
		// create local network taken from        
		// https://github.com/graphhopper/graphhopper/blob/0.5/core/src/test/java/com/graphhopper/GraphHopperTest.java#L746
		footEncoder = new FootFlagEncoder();       
		encodingManager = new EncodingManager(footEncoder);  
		weightings = new ArrayList<Weighting>(1);
		weightings.add(new FastestWeighting(footEncoder));
	}

	@Override
	public boolean createEdges(OSMReader reader, OSMWay way, TLongList osmNodeIds, long wayFlags, List<EdgeIteratorState> createdEdges) throws Exception 
	{
		if (!hasOpenSpace(way, osmNodeIds))
			return false;

		LongIntMap nodeMap = reader.getNodeMap();
		Polygon openSpace = osmPolygon2JTS(reader, osmNodeIds);

		internalTowerNodeIds.clear();
		intId2osmId.clear();
		idx2intId.clear();
		intId2idx.clear();

		// fill list with tower nodes        
		// fill map "internal ID 2 OSM ID"     
		for (int j = 0; j < osmNodeIds.size() - 1; j++) {       
			long osmNodeId = osmNodeIds.get(j);           
			int internalOSMId = reader.getNodeMap().get(osmNodeId);   
			intId2osmId.put(internalOSMId, osmNodeId);          
			if (internalOSMId < -2) //towernode
			{        
				internalTowerNodeIds.add(internalOSMId);    
			}      
		}

		DistanceCalc distCalc = Helper.DIST_EARTH;
		GraphHopperStorage graphStorage = new GraphHopperStorage(weightings,  new RAMDirectory(), encodingManager, false,  new GraphExtension.NoOpExtension()).create(20);

		for (int j = 0; j < osmNodeIds.size() - 1; j++) {                 
			long mainOsmId = osmNodeIds.get(j);        
			int internalMainId = nodeMap.get(mainOsmId);       
			int idxMain = j;         
			// coordinates of the first nodes     
			double latMain = reader.getTmpLatitude(internalMainId),      
					lonMain = reader.getTmpLongitude(internalMainId);     
			// connect the boundary of the open space        
			long neighborOsmId = osmNodeIds.get(j + 1);           
			int internalNeighborId = nodeMap.get(neighborOsmId);   
			int idxNeighbor = idxMain + 1;                               
			double latNeighbor = reader.getTmpLatitude(internalNeighborId), lonNeighbor = reader.getTmpLongitude(internalNeighborId);         
			double distance = distCalc.calcDist(latMain, lonMain, latNeighbor, lonNeighbor);    
			graphStorage.edge(idxMain, idxNeighbor, distance, true);         
			// iterate through remaining nodes,        
			// but not through the direct neighbors 
			for (int k = j + 2; k < osmNodeIds.size() - 1; k++) {  
				long partnerOsmId = osmNodeIds.get(k);          
				int internalPartnerId = nodeMap.get(partnerOsmId);  
				// coordinates of second nodes            
				double latPartner = reader.getTmpLatitude(internalPartnerId),
						lonPartner = reader.getTmpLongitude(internalPartnerId);   
				// connect nodes            
				LineString ls = (LineString) geometryFactory.createLineString( new Coordinate[] { new Coordinate(lonMain, latMain), new Coordinate(lonPartner, latPartner) }); 
				// check if new edge is within open space     
				if (ls.within(openSpace)) {        
					// compute distance between nodes        
					distance = distCalc.calcDist(latMain, lonMain, latPartner, lonPartner);    
					// the index number of the nodes in the local network                  
					// necessary, because it does not accept big values              
					int idxPartner = k;                   
					// fill             
					intId2idx.put(internalMainId, idxMain);  
					intId2idx.put(internalPartnerId, idxPartner);      
					// fill                   
					idx2intId.put(idxMain, internalMainId);     
					idx2intId.put(idxPartner, internalPartnerId);    
					// add edge to local graph          
					graphStorage.edge(idxMain, idxPartner, distance, true);                               
				}         
			}    
		}

		// a set with all created edges.  
		// the nodes which create the edge are stored in a ArrayList.   
		// it is important that the first node is smaller than the second node.  
		// TODO maybe a treeset would make the code more elegant
		_edges.clear();

		// compute routes between all tower nodes using the local graph    
		for (int i = 0; i < internalTowerNodeIds.size(); i++) {  
			int internalIdTowerStart = internalTowerNodeIds.get(i);   
			// check if tower node is in map         
			// it can miss if no edge is starting from here    
			if (false == intId2idx.containsKey(internalIdTowerStart)) {     
				continue;             }      
			int idxTowerStart = intId2idx.get(internalIdTowerStart);   
			for (int j = i + 1; j < internalTowerNodeIds.size(); j++) { 
				int internalIdTowerDestination = internalTowerNodeIds.get(j);      
				// check if tower node is in map          
				// it can miss if no edge is starting from here      
				if (false == intId2idx.containsKey(internalIdTowerDestination)) {    
					continue;                 }          
				int idxTowerDest = intId2idx.get(internalIdTowerDestination);  
				// compute route between tower nodes          
				try
				{
					Dijkstra dijkstra = new Dijkstra(graphStorage, footEncoder, weightings.get(0), TraversalMode.EDGE_BASED_2DIR);  
					Path path = dijkstra.calcPath(idxTowerStart, idxTowerDest,0);            
					TIntList pathNodes = path.calcNodes();           
					// iterate through nodes of routing result           
					for (int k = 0; k < pathNodes.size() - 1; k++) {      
						// local index                 
						int idxNodeA = pathNodes.get(k);            
						int idxNodeB = pathNodes.get(k + 1);            
						// internal Node IDs                
						int nodeA = idx2intId.get(idxNodeA);     
						int nodeB = idx2intId.get(idxNodeB);             
						// add to nodes to array sorted              
						int minNode = Integer.min(nodeA, nodeB);         
						int maxNode = Integer.max(nodeA, nodeB);            
						tmpEdge.clear(); 
						tmpEdge.add(minNode);          
						tmpEdge.add(maxNode);           
						boolean edgeIsNew = _edges.add(tmpEdge);        
						if (edgeIsNew) {     
							// it is necessary to get the long node OSM IDs...           
							long osmNodeA = intId2osmId.get(minNode);         
							long osmNodeB = intId2osmId.get(maxNode);            
							addNodePairAsEdgeToGraph(reader, way.getId(), wayFlags, createdEdges, osmNodeA, osmNodeB);           
						}     
					}
				}
				catch(Exception ex)
				{
					
				}
			}
		}

		// TODO this loop can maybe be integrated at the part where the boundary edges are handled alread<       
		// add boundary of open space
		for (int i = 0; i < osmNodeIds.size() - 1; i++) {     
			long osmIdA = osmNodeIds.get(i);        
			long osmIdB = osmNodeIds.get(i + 1);   
			int internalIdA = nodeMap.get(osmIdA);     
			int internalIdB = nodeMap.get(osmIdB);         
			// add to nodes to array sorted     
			int minIntId = Integer.min(internalIdA, internalIdB);    
			int maxIndId = Integer.max(internalIdA, internalIdB);
			// create a boundary edge     
			tmpEdge.clear();
			tmpEdge.add(minIntId);        
			tmpEdge.add(maxIndId);             
			// test if already exists       
			boolean edgeIsNew = _edges.add(tmpEdge);    
			if (edgeIsNew) {     
				// edge is added to global GraphHopper graph  
				addNodePairAsEdgeToGraph(reader, way.getId(), wayFlags, createdEdges, osmIdA, osmIdB);   
			}     
		}

		graphStorage.close();
		
		return true;
	}

	private void addNodePairAsEdgeToGraph(OSMReader reader, long wayOsmId, long wayFlags,  List<EdgeIteratorState> createdEdges, long Node1, long Node2) {   
		// list which contains the Nodes of the new Edge     
		TLongArrayList subgraphNodes = new TLongArrayList(5);  
		subgraphNodes.add(Node1);     
		subgraphNodes.add(Node2);      
		createdEdges.addAll(reader.addOSMWay(subgraphNodes, wayFlags, wayOsmId));   
	}

	private Polygon osmPolygon2JTS(OSMReader reader, TLongList osmNodeIds) {     
		// collect all coordinates in ArrayList       
		if (_coordinates == null || _coordinates.length < osmNodeIds.size())
			_coordinates = new Coordinate[osmNodeIds.size()];

		for (int i = 0; i < osmNodeIds.size(); i++) 
		{      
			long osmNodeId = osmNodeIds.get(i);       
			int internalID = reader.getNodeMap().get(osmNodeId);   
			_coordinates[i] = new Coordinate(reader.getTmpLongitude(internalID),  reader.getTmpLatitude(internalID));
		}  

		Coordinate[] coords  = Arrays.copyOf(_coordinates, osmNodeIds.size());
		LinearRing ring = geometryFactory.createLinearRing(coords);     
		LinearRing holes[] = null;    
		// a JTS polygon consists of a ring and holes   
		return geometryFactory.createPolygon(ring, holes);  
	}

	@Override
	public void finish() {
		
	}

	/* * checks if the OSM way is an open space      *      
	 * 
	 * @param way      
	 * @param osmNodeIds      
	 * @return      */ 
	private boolean hasOpenSpace(OSMWay way, TLongList osmNodeIds) 
	{    
		long firstNodeId = osmNodeIds.get(0);        
		long lastNodeId = osmNodeIds.get(osmNodeIds.size() - 1);       
		return (firstNodeId == lastNodeId) && way.hasTag("area", "yes");    
	} 

	@Override
	public String getName() {
		return "InField";
	}
}