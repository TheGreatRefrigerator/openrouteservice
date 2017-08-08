/*|----------------------------------------------------------------------------------------------
 *|														Heidelberg University
 *|	  _____ _____  _____      _                     	Department of Geography		
 *|	 / ____|_   _|/ ____|    (_)                    	Chair of GIScience
 *|	| |  __  | | | (___   ___ _  ___ _ __   ___ ___ 	(C) 2014-2017
 *|	| | |_ | | |  \___ \ / __| |/ _ \ '_ \ / __/ _ \	
 *|	| |__| |_| |_ ____) | (__| |  __/ | | | (_|  __/	Berliner Strasse 48								
 *|	 \_____|_____|_____/ \___|_|\___|_| |_|\___\___|	D-69120 Heidelberg, Germany	
 *|	        	                                       	http://www.giscience.uni-hd.de
 *|								
 *|----------------------------------------------------------------------------------------------*/
package heigit.ors.routing.algorithms;

import java.util.Arrays;
import java.util.PriorityQueue;

import com.carrotsearch.hppc.IntObjectMap;
import com.graphhopper.coll.GHIntObjectHashMap;
import com.graphhopper.routing.QueryGraph;
import com.graphhopper.routing.util.CHLevelEdgeFilter;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.util.TraversalMode;
import com.graphhopper.routing.weighting.Weighting;
import com.graphhopper.storage.CHGraph;
import com.graphhopper.storage.Graph;
import com.graphhopper.util.EdgeExplorer;
import com.graphhopper.util.EdgeIterator;

import heigit.ors.routing.graphhopper.extensions.storages.MultiTreeSPEntry;
import heigit.ors.util.DebugUtility;

public class RPHASTAlgorithm4 extends AbstractManyToManyRoutingAlgorithm {
	private IntObjectMap<MultiTreeSPEntry> _bestWeightMapFrom;
	private MultiTreeSPEntry _currFrom;
	private MultiTreeSPEntry _currTo;
	private PriorityQueue<MultiTreeSPEntry> _prioQueue;
	private CHLevelEdgeFilter _upwardEdgeFilter;
	private CHLevelEdgeFilter _downwardEdgeFilter;
	private SubGraph _targetGraph;
	private boolean _finishedFrom;
	private boolean _finishedTo;
	private int _visitedCountFrom;
	private int _visitedCountTo;
	private int numTrees;

	public RPHASTAlgorithm4(Graph graph, Weighting weighting, TraversalMode traversalMode) {
		super(graph, weighting, traversalMode);

		int size = Math.min(Math.max(200, graph.getNodes() / 10), 2000);

		initCollections(size);

		CHGraph chGraph = null;
		if (graph instanceof CHGraph)
			chGraph = (CHGraph) graph;
		else if (graph instanceof QueryGraph) {
			QueryGraph qGraph = (QueryGraph) graph;
			chGraph = (CHGraph) qGraph.getMainGraph();
		}

		setMaxVisitedNodes(Integer.MAX_VALUE);
		FlagEncoder encoder = weighting.getFlagEncoder();

		_upwardEdgeFilter = new CHLevelEdgeFilter(chGraph, encoder);
		_downwardEdgeFilter = new CHLevelEdgeFilter(chGraph, encoder);
		_downwardEdgeFilter.setBackwardSearch(true);

		inEdgeExplorer = graph.createEdgeExplorer();
		outEdgeExplorer = graph.createEdgeExplorer();
	}

	protected void initCollections(int size) {
		_prioQueue = new PriorityQueue<MultiTreeSPEntry>(size);
		_bestWeightMapFrom = new GHIntObjectHashMap<MultiTreeSPEntry>(size);
	}

	@Override
	public void reset() {
		_finishedFrom = false;
		_finishedTo = false;
		_prioQueue.clear();
		_bestWeightMapFrom.clear();
	}

	@Override
	public void prepare(int[] sources, int[] targets) {
		PriorityQueue<Integer> prioQueue = new PriorityQueue<>(100);
		this.numTrees = sources.length;

		// Phase I: build shortest path tree from all target nodes to the
		// highest node
		_targetGraph = new SubGraph(graph);

		addNodes(_targetGraph, prioQueue, targets);

		while (!prioQueue.isEmpty()) {
			int adjNode = prioQueue.poll();
			EdgeIterator iter = outEdgeExplorer.setBaseNode(adjNode);

			while (iter.next()) {
				if (!_downwardEdgeFilter.accept(iter))
					continue;

				_downwardEdgeFilter.updateHighestNode(iter);

				if (_targetGraph.addEdge(adjNode, iter, true))
					prioQueue.add(iter.getAdjNode());
			}
		}

		if (DebugUtility.isDebug())
			_targetGraph.print();
	}

	private void addNodes(SubGraph graph, PriorityQueue<Integer> prioQueue, int[] nodes) {
		for (int i = 0; i < nodes.length; i++) {
			int nodeId = nodes[i];
			if (nodeId >= 0) {
				if (graph != null)
					graph.addEdge(nodeId, null, true);
				prioQueue.add(nodeId);
			}
		}
	}

	protected void runUpwardSearch() {
		while (!isMaxVisitedNodesExceeded() && !_finishedFrom) {
			_finishedFrom = !upwardSearch();
		}
	}

	protected void runDownwardSearch() {
		while (!_finishedTo) {
			_finishedTo = !downwardSearch();
		}
	}

	@Override
	public int getVisitedNodes() {
		return _visitedCountFrom + _visitedCountTo;
	}

	private boolean upwardSearch() {
		if (_prioQueue.isEmpty())
			return false;

		_currFrom = _prioQueue.poll();
		fillEdgesUpward(_currFrom, _prioQueue, _bestWeightMapFrom, outEdgeExplorer);
		_visitedCountFrom++;

		return true;
	}

	private boolean downwardSearch() {
		if (_prioQueue.isEmpty())
			return false;

		_currTo = _prioQueue.poll();
		fillEdgesDownward(_currTo, _prioQueue, _bestWeightMapFrom, outEdgeExplorer);
		_visitedCountTo++;

		return true;
	}

	@Override
	public MultiTreeSPEntry[] calcPaths(int[] from, int[] to) {
		int[] edgeIds = new int[from.length];
		Arrays.fill(edgeIds, EdgeIterator.NO_EDGE);

		for (int i = 0; i < from.length; i++) {
			double[] newWeights = new double[from.length];
			Arrays.fill(newWeights, -1);
			newWeights[i] = 0;
			_currFrom = new MultiTreeSPEntry(edgeIds, from[i], newWeights);
			_currFrom.visited = true;
			_prioQueue.add(_currFrom);

			if (!traversalMode.isEdgeBased()) {
				_bestWeightMapFrom.put(from[i], _currFrom);
			} else
				throw new IllegalStateException("Edge-based behavior not supported");

		}

		outEdgeExplorer = graph.createEdgeExplorer();

		runUpwardSearch();
		_currFrom = _bestWeightMapFrom.get(_upwardEdgeFilter.getHighestNode());
		_currFrom.visited = true;
		_prioQueue.clear();
		_prioQueue.add(_currFrom);

		outEdgeExplorer = _targetGraph.createExplorer();

		runDownwardSearch();

		MultiTreeSPEntry[] targets = new MultiTreeSPEntry[to.length];

		for (int i = 0; i < to.length; i++)
			targets[i] = _bestWeightMapFrom.get(to[i]);

		return targets;
	}

	private void fillEdgesUpward(MultiTreeSPEntry currEdge, PriorityQueue<MultiTreeSPEntry> prioQueue,
			IntObjectMap<MultiTreeSPEntry> shortestWeightMap, EdgeExplorer explorer) {
		EdgeIterator iter = explorer.setBaseNode(currEdge.adjNode);

		if (iter == null) // we reach one of the target nodes
			return;

		while (iter.next()) {
			if (!_upwardEdgeFilter.accept(iter))
				continue;

			_upwardEdgeFilter.updateHighestNode(iter);

			MultiTreeSPEntry ee = shortestWeightMap.get(iter.getAdjNode());
			if (ee == null) {
				int[] edgeIds = new int[numTrees];
				Arrays.fill(edgeIds, EdgeIterator.NO_EDGE);
				ee = new MultiTreeSPEntry(edgeIds, iter.getAdjNode(), numTrees);
				double tmpWeight;
				for (int i = 0; i < numTrees; i++) {
					if (currEdge.weights[i] == -1)
						continue;

					tmpWeight = weighting.calcWeight(iter, false, currEdge.edge[i]) + currEdge.weights[i];
					if (Double.isInfinite(tmpWeight))
						continue;
					ee.weights[i] = tmpWeight;
					ee.parent[i] = currEdge;
					ee.edge[i] = iter.getEdge();
				}
				shortestWeightMap.put(iter.getAdjNode(), ee);
				prioQueue.add(ee);

			} else {
				double tmpWeight;

				boolean addToQ = false;
				for (int i = 0; i < numTrees; i++) {
					if (currEdge.weights[i] == -1)
						continue;

					tmpWeight = weighting.calcWeight(iter, false, currEdge.edge[i]) + currEdge.weights[i];
					if (Double.isInfinite(tmpWeight))
						continue;
					if (ee.weights[i] > tmpWeight || ee.weights[i] == -1) {
						ee.weights[i] = tmpWeight;
						ee.edge[i] = iter.getEdge();
						ee.parent[i] = currEdge;
						addToQ = true;
					}

				}
				if (addToQ) {
					prioQueue.add(ee);
				}

			}

		}
	}

	private void fillEdgesDownward(MultiTreeSPEntry currEdge, PriorityQueue<MultiTreeSPEntry> prioQueue,
			IntObjectMap<MultiTreeSPEntry> shortestWeightMap, EdgeExplorer explorer) {

		EdgeIterator iter = explorer.setBaseNode(currEdge.adjNode);

		if (iter == null)
			return;

		while (iter.next()) {
			MultiTreeSPEntry ee = shortestWeightMap.get(iter.getAdjNode());

			if (ee == null) {
				int[] edgeIds = new int[numTrees];
				Arrays.fill(edgeIds, EdgeIterator.NO_EDGE);
				ee = new MultiTreeSPEntry(edgeIds, iter.getAdjNode(), numTrees);
				ee.visited = true;
				double tmpWeight;
				for (int i = 0; i < numTrees; i++) {
					if (currEdge.weights[i] == -1)
						continue;

					tmpWeight = weighting.calcWeight(iter, false, currEdge.edge[i]) + currEdge.weights[i];
					if (Double.isInfinite(tmpWeight))
						continue;
					ee.weights[i] = tmpWeight;
					ee.parent[i] = currEdge;
					ee.edge[i] = iter.getEdge();

				}
				shortestWeightMap.put(iter.getAdjNode(), ee);

				prioQueue.add(ee);
			} else {
				double tmpWeight;
				boolean addToQ = false;
				for (int i = 0; i < numTrees; i++) {
					if (currEdge.weights[i] == -1)
						continue;

					tmpWeight = weighting.calcWeight(iter, false, currEdge.edge[i]) + currEdge.weights[i];
					if (Double.isInfinite(tmpWeight))
						continue;
					if (ee.weights[i] > tmpWeight || ee.weights[i] == -1) {
						ee.weights[i] = tmpWeight;
						ee.edge[i] = iter.getEdge();
						ee.parent[i] = currEdge;
						addToQ = true;
					}

				}
				if (ee.visited == false) {
					// // This is the case if the node has been assigned a
					// weight in
					// // the upwards pass (fillEdges). We need to use it in
					// the
					// // downwards pass to access lower level nodes, though
					// the
					// weight
					// // does not have to be reset necessarily //
					ee.visited = true;
					prioQueue.add(ee);
				}
				if (addToQ) {

					ee.visited = true;
					prioQueue.add(ee);

				}

			}
		}
	}
}