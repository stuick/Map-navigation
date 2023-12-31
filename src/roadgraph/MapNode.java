/**
 * A class to represent a node in the map
 */
package roadgraph;

import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

/**
 * @author UCSD MOOC development team
 * 
 * Class representing a vertex (or node) in our MapGraph
 *
 */
class MapNode implements Comparable<MapNode>
{
	/** The list of edges out of this node */
	private HashSet<MapEdge> edges;
		
	/** the latitude and longitude of this node */
	private GeographicPoint location;
	
	/** Record the total length of edges explore to get to this node */
	private double distanceFromStart;
	
	public double distFromEdge;
	
	public double distToDestination;
		
	/** 
	 * Create a new MapNode at a given Geographic location
	 * @param loc the location of this node
	 */
	MapNode(GeographicPoint loc)
	{
		location = loc;
		edges = new HashSet<MapEdge>();
		distanceFromStart = 0;
		distToDestination = 0;
	}
		
	/**
	 * Add an edge that is outgoing from this node in the graph
	 * @param edge The edge to be added
	 */
	void addEdge(MapEdge edge)
	{
		edges.add(edge);
	}
	
	/**  
	 * Return the neighbors of this MapNode 
	 * @return a set containing all the neighbors of this node
	 */
	Set<MapNode> getNeighbors(boolean motor)
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {

			if(!motor && edge.getRoadType().contains("motorway")) {
				System.out.println(edge.getRoadType() + ", " + edge.getRoadName());
				continue;
			}
			MapNode currNode = edge.getOtherNode(this);
			currNode.distFromEdge = edge.getLength();
			neighbors.add(currNode);
		}
		return neighbors;
	}
	
	/**  
	 * Return the neighbors of this MapNode
	 * @param totalDistance distance reach a this point of the traversal 
	 * @return a set containing all the neighbors of this node
	 */
	Set<MapNode> getNeighbors(double currentlength)
	{
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for (MapEdge edge : edges) {
			//double totalLength = currentlength + edge.getLength();
			MapNode currNode = edge.getOtherNode(this);
			
			//currNode.setDistanceFromStart(totalLength);
			neighbors.add(currNode);
		}
		return neighbors;
	}
	
	/**
	 * Get the geographic location that this node represents
	 * @return the geographic location of this node
	 */
	GeographicPoint getLocation()
	{
		return location;
	}
	
	/**
	 * return the edges out of this node
	 * @return a set contianing all the edges out of this node.
	 */
	Set<MapEdge> getEdges()
	{
		return edges;
	}
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode()
	{
		return location.hashCode();
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[(" + location + ")]";
//		toReturn += "Child: ";
//		for (MapEdge e: edges) {
//			GeographicPoint point = e.getEndPoint();
//			toReturn += e.getLength() + ", " + point + " || ";
//		}
//		toReturn += " ]";
		return toReturn;
	}
//	@Override
//	public String toString()
//	{
//		String toReturn = "[NODE at location (" + location + ")";
//		toReturn += " intersects streets: ";
//		for (MapEdge e: edges) {
//			toReturn += e.getRoadName() + ", ";
//		}
//		toReturn += "]";
//		return toReturn;
//	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (MapEdge e: edges) {
			toReturn += e.getRoadName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}
	
	/**
	 * Update the distance from the start during traversal
	 * @param distance the length to update
	 */
	public void setDistanceFromStart(double distance) {
		distanceFromStart = distance;
	}
	
	/**
	 * Retrieve the distance from the start during traversal
	 * @return the distance from the start
	 */
	public double getDistanceFromStart() {
		return distanceFromStart;
	}
	
	public void setDistToDestination(MapNode dest) {
		distToDestination = location.distance(dest.getLocation());
	}
	
	
	@Override
	public int compareTo(MapNode o) {		
		return Double.compare(this.distanceFromStart + distToDestination, o.getDistanceFromStart()+o.distToDestination);
	}

}
