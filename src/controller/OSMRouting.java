package controller;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.PriorityQueue;

import object.EdgeInfo;
import object.Geometry;
import object.NodeInfo;
import object.ToNodeInfo;


import global.OSMData;
import global.OSMParam;
import model.OSMInput;
import model.OSMOutput;

class OSMRouteParam {
	/**
	 * @param param
	 */
	static long START_NODE, END_NODE;
	static double START_LAT;
	static double START_LON;
	static double END_LAT;
	static double END_LON;
	
	static int START_TIME;
	static int DAY_INDEX;
	static int TIME_INTERVAL;
	static int TIME_RANGE;
	static int HEURISTIC_SPEED;
	static int FOUND_PATH_COUNT;
}

public class OSMRouting{
	
	public static void setParam(long startNode, long endNode,  
			int startTime, int dayIndex) {
			OSMRouteParam.START_NODE = startNode;
			OSMRouteParam.END_NODE = endNode;

			OSMRouteParam.START_TIME 			= startTime;
			OSMRouteParam.DAY_INDEX				= dayIndex;
			OSMRouteParam.TIME_INTERVAL 		= 15;
			OSMRouteParam.TIME_RANGE 			= 60;
			OSMRouteParam.HEURISTIC_SPEED		= 60;
			OSMRouteParam.FOUND_PATH_COUNT 		= 4;
		}
	
	/**
	 * routing using A* algorithm
	 * http://en.wikipedia.org/wiki/A*_search_algorithm
	 * @param startNode
	 * @param endNode
	 * @param startTime
	 * @param dayIndex
	 * @param pathNodeList return path
	 * @return
	 */
	public static double routingAStar(long startNode, long endNode, int startTime, int dayIndex, ArrayList<Long> pathNodeList) {
		// System.out.println("start finding the path...");
		int debug = 0;
		double totalCost = -1;
		try {
			// test store transversal nodes
			HashSet<Long> transversalSet = new HashSet<Long>();
			
			
			if (startNode == endNode) {
				System.out.println("start node is the same as end node.");
				return 0;
			}
			
			HashSet<Long> closedSet = new HashSet<Long>();
			HashMap<Long, NodeInfoHelper> nodeHelperCache = new HashMap<Long, NodeInfoHelper>();
		
			//Push the start node to priority queue
			PriorityQueue<NodeInfoHelper> openSet = new PriorityQueue<NodeInfoHelper>( 10000, new Comparator<NodeInfoHelper>() {
				public int compare(NodeInfoHelper n1, NodeInfoHelper n2) {
					return (int)(n1.getTotalCost() - n2.getTotalCost());
				}
			});
			//PriorityQueue<NodeInfoHelper> openSet = new PriorityQueue<NodeInfoHelper>();
			NodeInfoHelper current = new NodeInfoHelper(startNode);
			current.setCost(0);
			current.setCurrentLevel(10);
			current.setHeuristic(OSMRouting.estimateHeuristic(startNode, endNode));
			openSet.offer(current);	// push the start node
			nodeHelperCache.put(current.getNodeId(), current);	// add cache
		
			
			//Add the endnode to the queue
			HashSet<Long> endSet = new HashSet<Long>();
			endSet.add(endNode);
			
			current = null;
			
			int transverNodeCount = 1;
			while(!openSet.isEmpty()) {
				// remove current from openset
				current = openSet.poll();
				
				if(!transversalSet.contains(current.getNodeId())) {
					transversalSet.add(current.getNodeId());
					transverNodeCount++;
				}
				
				long nodeId = current.getNodeId();
				// add current to closedset
				closedSet.add(nodeId);
				if(endSet.contains(nodeId)) {	// find the destination
					//TODO Srikanth
					//current = current.getEndNodeHelper(endNode);
					totalCost = current.getCost();
					break;
				}
				// for time dependent routing
				int timeIndex = startTime + (int)(current.getCost() / OSMParam.SECOND_PER_MINUTE / OSMRouteParam.TIME_INTERVAL);
				if (timeIndex > OSMRouteParam.TIME_RANGE - 1)	// time [6am - 9 pm], we regard times after 9pm as constant edge weights
					timeIndex = OSMRouteParam.TIME_RANGE - 1;
				LinkedList<ToNodeInfo> adjNodeList = OSMData.adjListHashMap.get(nodeId);
				if(adjNodeList == null) continue;	// this node cannot go anywhere
				double arriveTime = current.getCost();
				// for each neighbor in neighbor_nodes(current)
				for(ToNodeInfo toNode : adjNodeList) {
					debug++;
					long toNodeId = toNode.getNodeId();
					int travelTime;
					if(toNode.isFix())	// fix time
						travelTime = toNode.getTravelTime();
					else	// fetch from time array
						travelTime = toNode.getSpecificTravelTime(dayIndex, timeIndex);
					// tentative_g_score := g_score[current] + dist_between(current,neighbor)
					double costTime = arriveTime + (double)travelTime / OSMParam.MILLI_PER_SECOND;
					// tentative_f_score := tentative_g_score + heuristic_cost_estimate(neighbor, goal)
					double heuristicTime =  estimateHeuristic(toNodeId, endNode);
					double totalCostTime = costTime + heuristicTime;
					// if neighbor in closedset and tentative_f_score >= f_score[neighbor]
					if(closedSet.contains(toNodeId) && nodeHelperCache.get(toNodeId).getTotalCost() <= totalCostTime) {
						continue;
					}
					NodeInfoHelper node = null;
					// if neighbor not in openset or tentative_f_score < f_score[neighbor]
					if(!nodeHelperCache.containsKey(toNodeId)) {	// neighbor not in openset
						node = new NodeInfoHelper(toNodeId);
						nodeHelperCache.put(node.getNodeId(), node);
					}
					else if (nodeHelperCache.get(toNodeId).getTotalCost() > totalCostTime) {	// neighbor in openset
						node = nodeHelperCache.get(toNodeId);
						if(closedSet.contains(toNodeId)) {	// neighbor in closeset
							closedSet.remove(toNodeId);	// remove neighbor form colseset
						}
						else {
							openSet.remove(node);
						}
					}

					// neighbor need update
					if(node != null) {
						node.setCost(costTime);
						node.setHeuristic(heuristicTime);
						node.setParentId(nodeId);
						openSet.offer(node);	// add neighbor to openset again
					}
				}
			}
			if(totalCost != -1) {
				long traceNodeId = current.getNodeId();
				pathNodeList.add(traceNodeId);	// add end node
				traceNodeId = current.getParentId();
				while(traceNodeId != 0) {
					pathNodeList.add(traceNodeId);	// add node
					current = nodeHelperCache.get(traceNodeId);
					traceNodeId = current.getParentId();
				}
				Collections.reverse(pathNodeList);	// reverse the path list
				// System.out.println("find the path successful!");
			}
			else {
				System.out.println("can not find the path!");
			}
			
			//OSMOutput.generateTransversalNodeKML(transversalSet);
			
			System.out.println("transveral node count: " + transverNodeCount);
		}
		catch(Exception e) {
			e.printStackTrace();
			System.err.println("tdsp: debug code " + debug + ", start node " + startNode + ", end node " + endNode);
		}
		return totalCost;
	}
	
	public static double estimateHeuristic(long curNode, long endNode) {
		NodeInfo cur = OSMData.nodeHashMap.get(curNode);
		NodeInfo end = OSMData.nodeHashMap.get(endNode);
		double distance = Geometry.calculateDistance(cur.getLocation(), end.getLocation());
		return distance / OSMRouteParam.HEURISTIC_SPEED * OSMParam.SECOND_PER_HOUR;
	}
	public static void tdsp() {
		// node list used for store path
		ArrayList<Long> pathNodeList = new ArrayList<Long>();
		
		// test start end node
		//OSMOutput.generateStartEndlNodeKML(START_NODE, END_NODE, nodeHashMap);
		// test count time
		long begintime = System.currentTimeMillis();
		
		
		
		double cost;
		// routing using A* algorithm with priority queue
		//cost = routingAStar(start.getNodeId(), end.getNodeId(), OSMRouteParam.START_TIME, OSMRouteParam.DAY_INDEX, pathNodeList);
		cost = routingAStar(OSMRouteParam.START_NODE, OSMRouteParam.END_NODE, OSMRouteParam.START_TIME, OSMRouteParam.DAY_INDEX, pathNodeList);
		
		// routing using A* algorithm with fibonacci heap
		//cost = routingAStarFibonacci(start.getNodeId(), end.getNodeId(), OSMRouteParam.START_TIME, OSMRouteParam.DAY_INDEX, pathNodeList);
		
		// routing using bidirectional hierarchy search
		//cost = routingHierarchy(start.getNodeId(), end.getNodeId(), OSMRouteParam.START_TIME, OSMRouteParam.DAY_INDEX, pathNodeList);
		//cost = routingHierarchy(54551130, 95582575, OSMRouteParam.START_TIME, OSMRouteParam.DAY_INDEX, pathNodeList);
		
		long endtime = System.currentTimeMillis();
		long response = (endtime - begintime);
		System.out.println("routing cost: " + cost + " s, response time: " + response + " ms");
		
		System.out.println("pathNodes:");
		for(int i=0;i<pathNodeList.size();i++)
			System.out.print(pathNodeList.get(i)+",");
		System.out.println();
		
		// output kml1
		OSMInput.readEdgeFile();
		OSMInput.addOnEdgeToNode();
		OSMOutput.generatePathKML(pathNodeList);
		//OSMOutput.generatePathNodeKML(pathNodeList);
		/*		
		String turnByTurn = turnByTurn(pathNodeList);
		System.out.println(turnByTurn);*/
	}
	
	public static void main(String[] args) {

		long startNode = Long.parseLong(args[0]);
		long endNode = Long.parseLong(args[1]);
		int startTime =Integer.parseInt (args[2]);
		int dayIndex =Integer.parseInt(args[3]);
		
		setParam(startNode,endNode, startTime, dayIndex);
		OSMParam.paramConfig(args[4],args[5]);
		OSMInput.readNodeFile();
		OSMInput.readNodeLocationGrid();
		// for both adjListHashMap and adjReverseListHashMap
		OSMInput.readAdjList();
		OSMParam.initialHierarchy();
		// JVM memory usage
		System.out.println("JVM total memory usage: " + Runtime.getRuntime().totalMemory() / (1024 * 1024) + " megabytes");
		
		// test
		//OSMOutput.generateHighwayKML(edgeHashMap, nodeHashMap);
		
		// initial hierarchy level
		OSMParam.initialHierarchy();
		
		// routing, turn by turn
		tdsp();
	}
}
/**
 * data structure used in queue
 */
class NodeInfoHelper {
	long nodeId;
	
	double cost;
	double heuristic;
	long parentId;
	
	// for hierarchy routing
	int currentLevel;
	
	public NodeInfoHelper(long nodeId) {
		this.nodeId = nodeId;
	}
	
	public long getNodeId() {
		return nodeId;
	}
	
	public void setCost(double cost) {
		this.cost = cost;
	}
	
	public double getCost() {
		return cost;
	}

	public void setHeuristic(double heuristic) {
		this.heuristic = heuristic;
	}

	public double getHeuristic() {
		return heuristic;
	}

	public double getTotalCost() {
		return cost + heuristic;
	}
	
	public void setParentId(long parentId) {
		this.parentId = parentId;
	}
	
	public long getParentId() {
		return parentId;
	}
	
	public void setCurrentLevel(int level) {
		currentLevel = level;
	}
	
	public int getCurrentLevel() {
		return currentLevel;
	}
	
//	public NodeInfoHelper getEndNodeHelper(long endNode) {
//		// TODO: not time dependent now, need to modify
//		NodeInfo end = OSMData.nodeHashMap.get(endNode);
//		NodeInfoHelper endNodeHelper = new NodeInfoHelper(endNode);
//		if(getNodeId() == end.getNodeId()) {	// no need to do in edge routing
//			return this;
//		}
//		else {
//			EdgeInfo edge = end.getOnEdgeList().getFirst();
//			int distance;
//			// from start to middle
//			if(getNodeId() == edge.getStartNode()) {
//				distance = edge.getStartDistance(endNode);
//			}
//			else {	// from end to middle
//				distance = edge.getEndDistance(endNode);
//			}
//			double speed  = edge.getTravelSpeed();
//			int travelTime = 1;	// second
//			travelTime = (int) Math.round(distance / speed);
//			endNodeHelper.setCost(getCost() + travelTime);
//			endNodeHelper.setParentId(getNodeId());
//		}
//		return endNodeHelper;
//	}
}