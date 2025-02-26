package TIMRAD_2025.helptool.object;


import rescuecore2.worldmodel.EntityID;

public class TIMRADHydrant {
	private EntityID selfID;
	private boolean occupied;

	public TIMRADHydrant(EntityID id) {
		this.selfID = id;
		this.occupied  = false;
	}
	public void update() {
		this.occupied = ! this.occupied;
	}
	
	public boolean isOccuped() {
		return this.occupied;
	}
	
	public String toString() {
		return "CSUH[" + this.selfID +"]";
	}

}
