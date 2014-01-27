package model;

import global.OSMData;
import global.OSMParam;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;

import object.EdgeInfo;
import object.LocationInfo;
import object.NodeInfo;
import object.ToNodeInfo;

public class OSMInput {
	/**
	 * read edge file
	 */
	public static void readEdgeFile() {
		System.out.println("read edge file...");
		int debug = 0;
		try {
			// from highway type to identity the index
			HashMap<String, Integer> highwayTypeCache = new HashMap<String, Integer>();
			String file = OSMParam.root + OSMParam.SEGMENT + OSMParam.edgeCVSFile;
			checkFileExist(file);
			FileInputStream fstream = new FileInputStream(file);
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;

			while ((strLine = br.readLine()) != null) {
				debug++;
				String[] nodes = strLine.split(OSMParam.ESCAPE_SEPARATION);
				long wayId = Long.parseLong(nodes[0]);
				int edgeId = Integer.parseInt(nodes[1]);
				String name = nodes[2];
				String highway = nodes[3];
				boolean isOneway = nodes[4].equals(OSMParam.ONEDIRECT);
				String nodeListStr = nodes[5];
				int distance = Integer.parseInt(nodes[6]);
				
				String[] nodeIds = nodeListStr.split(OSMParam.COMMA);
				LinkedList<Long> nodeList = new LinkedList<Long>();
				for(String nodeStr : nodeIds) {
					nodeList.add(Long.parseLong(nodeStr));
				}
				int highwayId;
				if(highwayTypeCache.containsKey(highway)) {
					highwayId = highwayTypeCache.get(highway);
				}
				else {
					highwayId = OSMData.edgeHighwayTypeList.size();
					highwayTypeCache.put(highway, highwayId);
					// add to highway list
					OSMData.edgeHighwayTypeList.add(highway);
				}
				EdgeInfo edge = new EdgeInfo(wayId, edgeId, name, highwayId, isOneway, nodeList, distance);
				OSMData.edgeHashMap.put(edge.getId(), edge);
			}
			br.close();
			in.close();
			fstream.close();
		} catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
			System.err.println("readEdgeFile: debug code: " + debug);
		}
		System.out.println("read edge file finish!");
	}
	public static void addOnEdgeToNode() {
		System.out.println("preparing nodes on edge...");
		for(EdgeInfo edge : OSMData.edgeHashMap.values()) {
			LinkedList<Long> nodeList = edge.getNodeList();
			for(long nodeId : nodeList) {
				NodeInfo node = OSMData.nodeHashMap.get(nodeId);
				node.addOnEdge(edge);
			}
		}
		System.out.println("preparing nodes on edge finish!");
	}
	/**
	 * cache for from lot and lon to node
	 */
	public static void readNodeLocationGrid() {
		System.out.println("preparing for nodes lat and lon...");
		for(NodeInfo node : OSMData.nodeHashMap.values()) {
			LocationInfo location = node.getLocation();
			double lat = location.getLatitude();
			double lon = location.getLongitude();
			DecimalFormat df=new DecimalFormat("0.0");
			String latLonId = df.format(lat) + OSMParam.COMMA + df.format(lon);
			//System.out.println(latLonId);
			LinkedList<NodeInfo> nodeList;
			if(OSMData.nodeLocationGridMap.containsKey(latLonId)) {
				nodeList = OSMData.nodeLocationGridMap.get(latLonId);
			}
			else {
				nodeList = new LinkedList<NodeInfo>();
				OSMData.nodeLocationGridMap.put(latLonId, nodeList);
			}
			nodeList.add(node);
		}
		System.out.println("preparing for nodes lat and lon finish!");
	}
	/**
	 * read the node file
	 */
	public static void readNodeFile() {
		System.out.println("read node file...");
		int debug = 0;
		try {
			String file = OSMParam.root + OSMParam.SEGMENT + OSMParam.nodeCSVFile;
			checkFileExist(file);
			FileInputStream fstream = new FileInputStream(file);
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;
			
			while ((strLine = br.readLine()) != null) {
				debug++;
				String[] nodes = strLine.split(OSMParam.ESCAPE_SEPARATION);
				long nodeId = Long.parseLong(nodes[0]);
				double latitude = Double.parseDouble(nodes[1]);
				double longitude = Double.parseDouble(nodes[2]);
				LocationInfo locationInfo = new LocationInfo(latitude, longitude);
				NodeInfo nodeInfo = new NodeInfo(nodeId, locationInfo);
				
				OSMData.nodeHashMap.put(nodeId, nodeInfo);
			}
			br.close();
			in.close();
			fstream.close();
		} catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
			System.err.println("readNodeFile: debug code: " + debug);
		}
		System.out.println("read node file finish!");
	}
	/**
	 * check whether file exist
	 * @param file
	 */
	public static void checkFileExist(String file) {
		File f = new File(file);
		
		if(!f.exists()) {
			System.err.println("Can not find " + file + ", program exit!");
			System.exit(-1);
		}
	}
	/**
	 * build adjList and adjReverseList from adjlist.csv for 7 days
	 */
	public static void readAdjList() {
		System.out.println("loading adjlist file: " + OSMParam.adjlistFile + "...");

		int debug = 0;
		try {
			String file = OSMParam.root + OSMParam.SEGMENT + OSMParam.adjlistFile;
			checkFileExist(file);
			FileInputStream fstream = new FileInputStream(file);
			DataInputStream in = new DataInputStream(fstream);
			BufferedReader br = new BufferedReader(new InputStreamReader(in));
			String strLine;

			System.out.println("building graph...");

			while ((strLine = br.readLine()) != null) {
				debug++;
				//if (debug % 100000 == 0)
				//	System.out.println("completed " + debug + " lines.");

				String[] nodes = strLine.split(OSMParam.ESCAPE_SEPARATION);
				long startNode = Long.parseLong(nodes[0]);

				LinkedList<ToNodeInfo> toNodeList = new LinkedList<ToNodeInfo>();
				String[] adjlists = nodes[1].split(OSMParam.SEMICOLON);
				for (int i = 0; i < adjlists.length; i++) {
					String adjComponent = adjlists[i];
					long toNode = Long.parseLong(adjComponent.substring(0, adjComponent.indexOf('(')));
					String fixStr = adjComponent.substring(adjComponent.indexOf('(') + 1, adjComponent.indexOf(')'));
					if (fixStr.equals(OSMParam.FIX)) { // fixed
						int travelTime = Integer.parseInt(adjComponent.substring(adjComponent.indexOf(OSMParam.COLON) + 1));
						ToNodeInfo toNodeInfo = new ToNodeInfo(toNode, travelTime);
						toNodeInfo.setMinTravelTime(travelTime);
						toNodeList.add(toNodeInfo);
						// build the reverse adjlist
						LinkedList<ToNodeInfo> fromNodeList;
						if(OSMData.adjReverseListHashMap.containsKey(toNode)) {
							fromNodeList = OSMData.adjReverseListHashMap.get(toNode);
						}
						else {
							fromNodeList = new LinkedList<ToNodeInfo>();
							OSMData.adjReverseListHashMap.put(toNode, fromNodeList);
						}
						ToNodeInfo fromNodeInfo = new ToNodeInfo(startNode, travelTime);
						fromNodeInfo.setMinTravelTime(travelTime);
						fromNodeList.add(fromNodeInfo);
					} else { // variable
						String timeList = adjComponent.substring(adjComponent.indexOf(OSMParam.COLON) + 1);
						String[] timeValueList = timeList.split(OSMParam.COMMA);
						int minTravelTime = Integer.MAX_VALUE;
						ArrayList<ArrayList<Integer>> travelTimeArray = new ArrayList<ArrayList<Integer>>();
						for(int day = 0; day < 7; day++) {
							ArrayList<Integer> array = new ArrayList<Integer>();
							for(int j = day * 60; j < (day + 1) * 60 && j < timeValueList.length; j++) {
								int time = Integer.parseInt(timeValueList[j]);
								array.add(time);
								if(time < minTravelTime)
									minTravelTime = time;
							}
							travelTimeArray.add(array);
						}
						ToNodeInfo toNodeInfo = new ToNodeInfo(toNode, travelTimeArray);
						toNodeInfo.setMinTravelTime(minTravelTime);
						toNodeList.add(toNodeInfo);
						// build the reverse adjlist
						LinkedList<ToNodeInfo> fromNodeList;
						if(OSMData.adjReverseListHashMap.containsKey(toNode)) {
							fromNodeList = OSMData.adjReverseListHashMap.get(toNode);
						}
						else {
							fromNodeList = new LinkedList<ToNodeInfo>();
							OSMData.adjReverseListHashMap.put(toNode, fromNodeList);
						}
						ToNodeInfo fromNodeInfo = new ToNodeInfo(startNode, travelTimeArray);
						fromNodeInfo.setMinTravelTime(minTravelTime);
						fromNodeList.add(fromNodeInfo);
					}
				}
				OSMData.adjListHashMap.put(startNode, toNodeList);
			}

			br.close();
			in.close();
			fstream.close();
		} catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
			System.err.println("buildList: debug code: " + debug);
		}
		System.out.println("building graph finish!");
	}
}
