package roadgraph;

/**
 * A class to find the time to travel from one Node to the next
 * @author STUICK
 *
 */
public class NodeTravel {

	//
	private MapNode prev;
	private MapNode curr;
	private double travelTime;
	private double travelSpeed;
	private double distance;
	
	public NodeTravel (MapNode curr) {
		prev = null;
		this.curr = curr;
		travelTime = 0;
		travelSpeed = 0;
		distance =0;
	}
		
	public NodeTravel(MapNode prev, MapNode curr, double travelSpeed) {
		this.prev = prev;
		this.curr = curr;
		this.travelSpeed = travelSpeed;
	}
	
	public double getTravelTime() {
		distance = getDistance();
		travelTime = distance / travelSpeed;
		return travelTime;
	}
	
	public double getDistance() {
		return prev.getLocation().distance(curr.getLocation());
	}
}
