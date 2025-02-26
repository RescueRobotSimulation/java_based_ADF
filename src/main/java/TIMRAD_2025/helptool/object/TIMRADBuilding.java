package TIMRAD_2025.helptool.object;


import TIMRAD_2025.debugger.DebugHelper;
import TIMRAD_2025.module.complex.fb.tools.FbUtilities;
import TIMRAD_2025.helptool.Util;
import TIMRAD_2025.helptool.TIMRADWorldHelper;
import adf.core.launcher.ConfigKey;
import javolution.util.FastSet;
import rescuecore2.log.Logger;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.geom.Line2D;
import java.io.Serializable;
import java.util.*;

public class TIMRADBuilding {
	public static Map<EntityID, Map<EntityID, TIMRADBuilding>> VIEWER_BUILDING_MAP = new HashMap<>();

	private List<EntityID> areasInExtinguishableRange;


	private List<TIMRADBuilding> radiationNeighbourBuildings;

	private List<EntityID> radiationNeighbourBuildingsId;

	private TIMRADLineOfSightPerception lineOfSightPerception;
	private List<EntityID> observableAreas;

	private List<TIMRADWall> walls;
	private double totalWallArea;
	private List<TIMRADWall> allWalls;

	private Hashtable<TIMRADBuilding, Integer> connectedBuildingTable;

	private List<TIMRADBuilding> connectedBuildings;

	private List<Float> connectedValues;
	private Building selfBuilding;
	private TIMRADWorldHelper worldHelper;

	private boolean visited;

	private Set<TIMRADBuilding> neighbourDangerBuildings;

	public double BUILDING_VALUE = Double.MIN_VALUE;///
	public double priority =Double.MIN_VALUE;
	private int ignitionTime = -1;

	private int zoneId;

	private double advantageRatio;

	private boolean isVisible = false;
	private double hitRate = 0;
	protected int totalHits;
	private int lastSeenTime;
	private int lastUpdateTime;


	public static final EnumSet<StandardEntityConstants.Fieryness> ESTIMATED_BURNING = EnumSet
			.of(StandardEntityConstants.Fieryness.HEATING,
					StandardEntityConstants.Fieryness.BURNING,
					StandardEntityConstants.Fieryness.INFERNO);
	private Set<EntityID> visibleFrom;
	private List<TIMRADLineOfSightPerception.TIMRADRay> lineOfSight;

	public TIMRADBuilding() {
	};

	public TIMRADBuilding(StandardEntity entity, TIMRADWorldHelper worldHelper) {
		this.worldHelper = worldHelper;
		this.selfBuilding = (Building) entity;
		this.connectedBuildingTable = new Hashtable<TIMRADBuilding, Integer>(30);
		this.connectedBuildings = new ArrayList<TIMRADBuilding>();
		this.connectedValues = new ArrayList<Float>();

		this.radiationNeighbourBuildings = new ArrayList<TIMRADBuilding>();
		this.radiationNeighbourBuildingsId = new ArrayList<EntityID>();
		this.neighbourDangerBuildings = new FastSet<TIMRADBuilding>();

		this.lineOfSightPerception = new TIMRADLineOfSightPerception(worldHelper);

		if (DebugHelper.DEBUG_MODE && !worldHelper.getScenarioInfo().getRawConfig().getBooleanValue(ConfigKey.KEY_PRECOMPUTE, false)) {
			Pair<Integer, Integer> location = worldHelper.getLocation(selfBuilding.getID());
			Point2D point2D = new Point2D(location.first(), location.second());
			Set<TIMRADLineOfSightPerception.TIMRADRay> allRays = lineOfSightPerception.findRaysNotHit(point2D, new HashSet<>());
			ArrayList<Line2D> elements = new ArrayList<>();
			for (TIMRADLineOfSightPerception.TIMRADRay allRay : allRays) {
				rescuecore2.misc.geometry.Line2D ray = allRay.getRay();
				elements.add(Util.convertLine2(ray));
			}
			DebugHelper.VD_CLIENT.drawAsync(entity.getID().getValue(), "LineOfSightLayer", elements);
		}

		this.visited = false;
		this.visibleFrom = new HashSet<>();

		this.initWalls(worldHelper);
		this.initSimulatorValues();
	}

	public void addNeighbourBuilding(TIMRADBuilding neighbour) {
		this.radiationNeighbourBuildings.add(neighbour);
		this.radiationNeighbourBuildingsId.add(neighbour.getSelfBuilding()
				.getID());
		this.allWalls.addAll(neighbour.getWalls());
	}

	private void initWalls(TIMRADWorldHelper worldHelper) {
		int[] apexList = this.selfBuilding.getApexList();
		int firstX = apexList[0], firstY = apexList[1];
		int lastX = firstX, lastY = firstY;
		TIMRADWall wall;
		this.walls = new ArrayList<>();
		this.allWalls = new ArrayList<>();

		for (int i = 2; i < apexList.length; i++) {
			int tempX = apexList[i], tempY = apexList[++i];
			wall = new TIMRADWall(lastX, lastY, tempX, tempY, this);
			if (wall.validate()) {
				this.walls.add(wall);
				this.totalWallArea += FLOOR_HEIGHT * wall.length * 1000;
			} else {
				Logger.warn("Ignoring odd wall at building "
						+ selfBuilding.getID().getValue());
			}
			lastX = tempX;
			lastY = tempY;
		}
		wall = new TIMRADWall(lastX, lastY, firstX, firstY, this);
		if (wall.validate()) {
			this.walls.add(wall);
		}
		allWalls.addAll(walls);
		this.totalWallArea = this.totalWallArea / 1000000d;
	}


	public void initWallValue(TIMRADWorldHelper worldHelper) {
		int totalRays = 0; // total number of rays this building emitted
		for (TIMRADWall wall : this.walls) {
			wall.findHits(this);
			totalHits += wall.hits;
			totalRays += wall.rays;
		}

		TIMRADBuilding building;
		for (Enumeration<TIMRADBuilding> e = this.connectedBuildingTable.keys(); e
				.hasMoreElements();) {
			building = (TIMRADBuilding) e.nextElement();
			float value = this.connectedBuildingTable.get(building);
			this.connectedBuildings.add(building);
			this.connectedValues.add(value / (float) totalRays);
		}
		hitRate = totalHits * 1.0 / totalRays;
	}

	public double getBuildingRadiation() {
		double value = 0;
		TIMRADBuilding building;

		for (int i = 0; i < this.connectedValues.size(); i++) {
			building = this.connectedBuildings.get(i);
			if (building.isBurning()) {
				value += connectedValues.get(i);
			}
		}
		return value * this.getEstimatedTemperature() / 1000;
	}

	public double getNeighbourRadiation() {
		double value = 0;
		int index = 0;
		for (TIMRADBuilding building : this.radiationNeighbourBuildings) {
			index = building.getConnectedBuildings().indexOf(this);
			if (index >= 0) {
				value += building.getConnectedValues().get(index)
						* building.getEstimatedTemperature();
			}
		}
		return value / 1000;
	}

	public boolean isBurning() {
		return getEstimatedFieryness() > 0 && getEstimatedFieryness() < 4;
	}

	public List<TIMRADWall> getWalls() {
		return this.walls;
	}

	public List<TIMRADWall> getAllWalls() {
		return this.allWalls;
	}

	public List<EntityID> getAreasInExtinguishableRange() {
		if (areasInExtinguishableRange == null
				|| areasInExtinguishableRange.isEmpty()) {
			areasInExtinguishableRange = new ArrayList<>();
			int range = (int) (worldHelper.getConfig().extinguishableDistance * 0.9);
			for (StandardEntity next : worldHelper.getWorldInfo().getObjectsInRange(getId(), range)) {
				if (next instanceof Area)
					areasInExtinguishableRange.add(next.getID());
			}
		}

		return areasInExtinguishableRange;
	}

	public Set<TIMRADBuilding> getNeighbourDangerBuildings() {
		neighbourDangerBuildings = new HashSet<>();
		for(EntityID id : this.selfBuilding.getNeighbours()) {
			if(worldHelper.getWorldInfo().getEntity(id) instanceof Building) {
				Building build = (Building) worldHelper.getWorldInfo().getEntity(id);
				if(build.isOnFire()) ///|| build.isTemperatureDefined() && build.getTemperature() > 35)
					neighbourDangerBuildings.add(worldHelper.getCsuBuilding(build));
			}
		}
		return neighbourDangerBuildings;
	}

	
	public List<TIMRADBuilding> getRadiationNeighbourBuildings() {
		return radiationNeighbourBuildings;
	}

	public List<EntityID> getRadiationNeighbourBuildinsId() {
		return radiationNeighbourBuildingsId;
	}

	public Hashtable<TIMRADBuilding, Integer> getConnectedBuildingTable() {
		return connectedBuildingTable;
	}

	public void setConnectedBuildingTable(
			Hashtable<TIMRADBuilding, Integer> connectedBuildingTable) {
		this.connectedBuildingTable = connectedBuildingTable;
	}

	public List<TIMRADBuilding> getConnectedBuildings() {
		return connectedBuildings;
	}

	public void setConnectedBuildins(List<TIMRADBuilding> connected) {
		this.connectedBuildings = connected;
	}

	public List<Float> getConnectedValues() {
		return connectedValues;
	}

	public void setConnectedValues(List<Float> connectedValues) {
		this.connectedValues = connectedValues;
	}

	
	public int getIgnitionTime() {
		return ignitionTime;
	}

	public void setIgnitionTime(int ignitionTime) {
		this.ignitionTime = ignitionTime;
	}

	public boolean isVisited() {
		return this.visited;
	}

	public void setVisited(boolean visited) {
		this.visited = visited;
	}

	public List<EntityID> getObservableAreas() {
		if (observableAreas == null || observableAreas.isEmpty()) {
			observableAreas = lineOfSightPerception.getVisibleAreas(getId());
		}

		return observableAreas;
	}

	static final int FLOOR_HEIGHT = 3;
	static float RADIATION_COEFFICIENT = 0.011f;
	static final double STEFAN_BOLTZMANN_CONSTANT = 0.000000056704;

	private int startTime = -1;

	private float fuel;
	private float initFuel = -1;

	private float volume;
	private double energy;

	private float capacity;

	private float prevBurned;


	private int waterQuantity = 0;

	private int lwater = 0;
	private int lwTime = -1;
	private boolean wasEverWatered = false;
	private boolean inflammable = true;

	public static float woodIgnition = 47.0f;
	public static float steelIgnition = 47.0f;
	public static float concreteIgnition = 47.0f;

	public static float woodCapacity = 1.1f;
	public static float steelCapacity = 1.0f;
	public static float concreteCapacity = 1.5f;

	public static float woodEnergy = 2400.0f;
	public static float steelEnergy = 800.0f;
	public static float concreteEnergy = 350.0f;

	public static float woodBurning = 800.0f;
	public static float steelBurning = 850.0f;
	public static float concreteBurning = 800.0f;

	public void initSimulatorValues() {
		volume = selfBuilding.getGroundArea() * selfBuilding.getFloors()
				* FLOOR_HEIGHT;
		fuel = getInitialFuel();
		capacity = (volume * getThermoCapacity());
		energy = 0;
		initFuel = -1;
		prevBurned = 0;

		lwater = 0;
		lwTime = -1;
		wasEverWatered = false;

		Logger.info("Initialised the simulator values for building "
				+ selfBuilding.getID() + ": ground area = "
				+ selfBuilding.getGroundArea() + ", floors = "
				+ selfBuilding.getFloors() + ", volume = " + volume
				+ ", initial fuel = " + initFuel + ", energy capacity = "
				+ getCapacity());
	}

	public float getInitialFuel() {
		if (initFuel < 0)
			initFuel = getFuelDensity() * volume;
		return initFuel;
	}

	public float getThermoCapacity() {
		switch (selfBuilding.getBuildingCode()) {
		case 0:
			return woodCapacity;
		case 1:
			return steelCapacity;
		default:
			return concreteCapacity;
		}
	}

	public float getIgnitionPoint() {
		switch (selfBuilding.getBuildingCode()) {
		case 0:
			return woodIgnition;
		case 1:
			return steelIgnition;
		default:
			return concreteIgnition;
		}
	}

	public float getFuelDensity() {
		switch (selfBuilding.getBuildingCode()) {
		case 0:
			return woodEnergy;
		case 1:
			return steelEnergy;
		default:
			return concreteEnergy;
		}
	}

	public float getConsume(double burnRate) {
		if (fuel == 0)
			return 0;
		float tf = (float) (getEstimatedTemperature() / 1000f);
		float lf = fuel / getInitialFuel();
		float f = (float) (tf * lf * burnRate);
		if (f < 0.005f)
			f = 0.005f;
		return getInitialFuel() * f;

	}

	public double getEstimatedTemperature() {
		double rv = energy / capacity;

		if (Double.isNaN(rv)) {
			Logger.warn("Building " + selfBuilding.getID()
					+ " getTemperature returned NaN");
			new RuntimeException().printStackTrace();
			Logger.warn("Energy: " + energy);
			Logger.warn("Capacity: " + getCapacity());
			Logger.warn("Volume: " + volume);
			Logger.warn("Thermal capacity: " + getThermoCapacity());
			Logger.warn("Ground area: " + selfBuilding.getGroundArea());
			Logger.warn("Floors: " + selfBuilding.getFloors());

		}
		if (rv == Double.NaN || rv == Double.POSITIVE_INFINITY
				|| rv == Double.NEGATIVE_INFINITY)
			rv = Double.MAX_VALUE * 0.75;
		return rv;
	}

	public int getEstimatedFieryness() {
		if (!isInflammable())
			return 0;
		if (getEstimatedTemperature() >= getIgnitionPoint()) {
			if (fuel >= getInitialFuel() * 0.66)
				return 1; // HEATING
			if (fuel >= getInitialFuel() * 0.33)
				return 2; // BURNINGS
			if (fuel > 0)
				return 3; // INFERNO
		}
		if (fuel == getInitialFuel())
			if (wasEverWatered)
				return 4;
			else
				return 0;
		if (fuel >= getInitialFuel() * 0.66)
			return 5;
		if (fuel >= getInitialFuel() * 0.33)
			return 6;
		if (fuel > 0)
			return 7;
		return 8;
	}

	public double getRadiationEnergy() { // /273 293
		double t = this.getEstimatedTemperature() + 273;
		double radEn = (t * t * t * t) * RADIATION_COEFFICIENT
				* STEFAN_BOLTZMANN_CONSTANT * totalWallArea;

		if (selfBuilding.getID().getValue() == 23545) {
			Logger.debug("Getting radiation energy for building "
					+ selfBuilding.getID().getValue());
			Logger.debug("t = " + t);
			Logger.debug("t^4 = " + (t * t * t * t));
			Logger.debug("Total wall area: " + totalWallArea);
			Logger.debug("Radiation coefficient: " + RADIATION_COEFFICIENT);
			Logger.debug("Stefan-Boltzmann constant: "
					+ STEFAN_BOLTZMANN_CONSTANT);
			Logger.debug("Radiation energy: " + radEn);
			Logger.debug("Building energy: " + getEnergy());
		}

		if (radEn == Double.NaN || radEn == Double.POSITIVE_INFINITY
				|| radEn == Double.NEGATIVE_INFINITY)
			radEn = Double.MAX_VALUE * 0.75;
		if (radEn > getEnergy()) {
			radEn = getEnergy();
		}
		return radEn;
	}

	public Building getSelfBuilding() {
		return selfBuilding;
	}

	public float getVolum() {
		return this.volume;
	}

	public float getCapacity() {
		return this.capacity;
	}

	public int getRealFieryness() {
		return this.selfBuilding.getFieryness();
	}

	public int getRealTemperature() {
		return this.selfBuilding.getTemperature();
	}

	public double getEnergy() {
		if (energy == Double.NaN || energy == Double.POSITIVE_INFINITY
				|| energy == Double.NEGATIVE_INFINITY)
			energy = Double.MAX_VALUE * 0.75d;

		return this.energy;
	}

	public void setEnergy(double value, String invokeMethod) {
		if (value == Double.NaN || value == Double.POSITIVE_INFINITY
				|| value == Double.NEGATIVE_INFINITY)
			value = Double.MAX_VALUE * 0.75d;

		this.energy = value;
	}

	public float getPrevBurned() {
		return this.prevBurned;
	}

	public void setPrevBurned(float consumed) {
		this.prevBurned = consumed;
	}

	public int getWaterQuantity() {
		return this.waterQuantity;
	}

	public void setWaterQuantity(int i) {
		if (i > this.waterQuantity) {
			this.lwTime = worldHelper.getAgentInfo().getTime();
			this.lwater = i - waterQuantity;
			this.wasEverWatered = true;
		}
		this.waterQuantity = i;
	}

	public int getLastWater() {
		return lwater;
	}

	public boolean getLastWatered() {
		return lwTime == worldHelper.getAgentInfo().getTime()-1;
	}

	public void setWasEverWatered(boolean wasEverWatered) {
		this.wasEverWatered = wasEverWatered;
	}

	public float getFuel() {
		return this.fuel;
	}

	public void setFuel(float fuel) {
		this.fuel = fuel;
	}

	public boolean isInflammable() {
		return this.inflammable;
	}

	public void setInflammable(boolean inflammable) {
		this.inflammable = inflammable;
	}

	public int getStartTime() {
		return this.startTime;
	}

	public void setStartTime(int startTime) {
		this.startTime = startTime;
	}

	public boolean isEstimatedOnFire() {
		return ESTIMATED_BURNING.contains(getEstimatedFieryness());
	}

	private float tempFuel;
	private double tempEnergy;
	private float tempPrevBurned;
	private boolean tempInflammable;

	public void initTempSimulatorValues() {
		tempFuel = fuel;
		tempEnergy = energy;
		tempPrevBurned = prevBurned;
		tempInflammable = inflammable;
	}

	public float getTempConsume(double bRate) {
		if (tempFuel == 0) {
			return 0;
		}
		float tf = (float) (getTempEstimatedTemperature() / 1000f);
		float lf = tempFuel / getInitialFuel();
		float f = (float) (tf * lf * bRate);
		if (f < 0.005f)
			f = 0.005f;
		return getInitialFuel() * f;
	}

	public float getTempFlue(double burnRate) {
		if (tempFuel == 0) {
			return 0;
		}
		float tf = (float) (getTempEstimatedTemperature() / 1000f);
		float lf = tempFuel / getInitialFuel();
		float f = (float) (tf * lf * burnRate);
		if (f < 0.005f)
			f = 0.005f;
		return getInitialFuel() * f;
	}

	public double getTempEstimatedTemperature() {
		double rv = tempEnergy / capacity;
		if (Double.isNaN(rv)) {
			new RuntimeException().printStackTrace();
		}
		if (rv == Double.NaN || rv == Double.POSITIVE_INFINITY
				|| rv == Double.NEGATIVE_INFINITY)
			rv = Double.MAX_VALUE * 0.75;
		return rv;
	}

	public int getTempEstimatedFieryness() {
		if (!isInflammable())
			return 0;
		if (getTempEstimatedTemperature() >= getIgnitionPoint()) {
			if (tempFuel >= getInitialFuel() * 0.66)
				return 1; // burning, slightly damaged
			if (tempFuel >= getInitialFuel() * 0.33)
				return 2; // burning, more damaged
			if (tempFuel > 0)
				return 3; // burning, severly damaged
		}
		if (tempFuel == getInitialFuel())
			if (wasEverWatered)
				return 4; // not burnt, but watered-damaged
			else
				return 0; // not burnt, no water damage
		if (tempFuel >= getInitialFuel() * 0.66)
			return 5; // extinguished, slightly damaged
		if (tempFuel >= getInitialFuel() * 0.33)
			return 6; // extinguished, more damaged
		if (tempFuel > 0)
			return 7; // extinguished, severely damaged
		return 8; // completely burnt down
	}

	public double getTempRadiationEnergy() { // /273.15
		double t = getTempEstimatedTemperature() + 293;
		double radEn = (t * t * t * t) * totalWallArea * RADIATION_COEFFICIENT
				* STEFAN_BOLTZMANN_CONSTANT;
		if (radEn == Double.NaN || radEn == Double.POSITIVE_INFINITY
				|| radEn == Double.NEGATIVE_INFINITY)
			radEn = Double.MAX_VALUE * 0.75;
		if (radEn > tempEnergy) {
			radEn = tempEnergy;
		}
		return radEn;
	}

	public float getTempFuel() {
		return this.tempFuel;
	}

	public void setTempFuel(float tempFuel) {
		this.tempFuel = tempFuel;
	}

	public double getTempEnergy() {
		return this.tempEnergy;
	}

	public void setTempEnergy(double tempEnergy) {
		this.tempEnergy = tempEnergy;
	}

	public float getTempPrevBurned() {
		return this.tempPrevBurned;
	}

	public void setTempPrevBurned(float tempPrevBurned) {
		this.tempPrevBurned = tempPrevBurned;
	}

	public boolean isTempFlammable() {
		return this.tempInflammable;
	}

	public void setTempFlammable(boolean tempFlammable) {
		this.tempInflammable = tempFlammable;
	}

	public boolean wasEverWatered() {
		return this.wasEverWatered;
	}
	public boolean isExtinguished() {
		return this.getEstimatedFieryness() >= 4
				&& this.getEstimatedFieryness() < 8;
	}

	public boolean isCollapsed() {
		return this.getEstimatedFieryness() == 8;
	}

	public EntityID getId() {
		return this.selfBuilding.getID();
	}

	public TIMRADWorldHelper getWorldModel() {
		return this.worldHelper;
	}

	public double getBuildingAreaTempValue() {
		double areaTempValue = this.selfBuilding.getTotalArea()
				* this.getEstimatedTemperature();
		return Util.gauss2mf(areaTempValue, 10000, 30000, 20000, 40000);
	}

	public Integer getZoneId() {
		return zoneId;
	}

	public void setZoneId(Integer zoneId) {
		this.zoneId = zoneId;
	}

	public boolean isBurned() {
		return getEstimatedFieryness() == 8;
	}

	public double getAdvantageRatio() {
		return this.advantageRatio;
	}

	public void setAdvantageRatio(double advantageRatio) {
		this.advantageRatio = advantageRatio;
	}

	public void setVisible(boolean visible) {
		this.isVisible = visible;
	}

	public boolean isVisible() {
		return this.isVisible;
	}

	@Override
	public String toString() {
		return "CSUBuilding: [" + this.selfBuilding + "]";
	}

	public boolean isBigBuilding() {
		int sum = 0;
		int num = 0;
		for(Building build : worldHelper.getEntitiesOfType(Building.class, StandardEntityURN.BUILDING)) {
			num++;
			sum += build.getGroundArea();
		}
		int avg = sum / num;
		return this.selfBuilding.getGroundArea() > Math.min(2300, 2*avg);
		
	}

	public int getExtinguishableCycle() {
		int waterNeeded = FbUtilities.waterNeededToExtinguish(this);
		int cycle = waterNeeded / worldHelper.getConfig().maxPower;
		return cycle;
	}

	public void addPriority(int value) {
		this.priority += value;
	}
	public void addValue(double value) {
		this.BUILDING_VALUE += value;
	}
	
	public boolean isOutFire() {
		if(worldHelper.getBorderBuildings().contains(worldHelper.getEntity(this.getId(), Building.class)))
			return false;
		
		if(! this.inflammable || ! this.isBurning())
			return false;
		Set<TIMRADBuilding> fireConnectedBuildings = new HashSet<>();
		for(TIMRADBuilding build : this.getConnectedBuildings()) {
			if(build.isBurning()) 
				fireConnectedBuildings.add(build);
		}
	
		int i = 0;
		int result0 = 0;
		int result = 0;
		int changed = 0;
		
		for(TIMRADBuilding build : fireConnectedBuildings) {
			if(i++ == 0)
				result0 = isOneVerticalSide(this, build);
			result = isOneVerticalSide(this, build);
			if(result != result0) {
				changed = 1;
				break;
			}
		}
		if(changed == 0)
			return true;
		
		i=0;
		result0 =0;
		result = 0; 
		changed = 0;
		
		for(double rad = -9/20 * Math.PI; rad <= 9/20 * Math.PI; rad += Math.PI/10) {
			for(TIMRADBuilding build : fireConnectedBuildings) {
				if(i++ == 0)
					result0 = underLine(this, build, rad);
				result = underLine(this, build, rad);
				if(result != result0) {
					changed = 1;
					break;
				}
			}
			if(changed == 0)
				return true;
			changed = 0;
		}
		return false;
	}
		
	public int isOneVerticalSide(TIMRADBuilding one, TIMRADBuilding two) {
		int x1 = one.selfBuilding.getX();
		int x2 = two.selfBuilding.getX();
		if(Math.abs(x1-x2) < 8000)
			return 0;
		else 
			return 1;
	}
	
	public int underLine(TIMRADBuilding one, TIMRADBuilding two, double k) {
		int x1 = one.selfBuilding.getX();
		int y1 = one.selfBuilding.getY();
		int x2 = two.selfBuilding.getX();
		int y2 = two.selfBuilding.getY();
		double result = k * (x2 - x1) + y1 - y2;
		if(result <= 0)
			return 0;
		else 
			return 1;
	}

	public void updateValues(Building building) {
		switch (building.getFieryness()) {
			case 0:
				this.setFuel(this.getInitialFuel());
				if (getEstimatedTemperature() >= getIgnitionPoint()) {
					setEnergy(getIgnitionPoint() / 2, "updateValues");
				}
				break;
			case 1:
				if (getFuel() < getInitialFuel() * 0.66) {
					setFuel((float) (getInitialFuel() * 0.75));
				} else if (getFuel() == getInitialFuel()) {
					setFuel((float) (getInitialFuel() * 0.90));
				}
				break;

			case 2:
				if (getFuel() < getInitialFuel() * 0.33
						|| getFuel() > getInitialFuel() * 0.66) {
					setFuel((float) (getInitialFuel() * 0.50));
				}
				break;

			case 3:
				if (getFuel() < getInitialFuel() * 0.01
						|| getFuel() > getInitialFuel() * 0.33) {
					setFuel((float) (getInitialFuel() * 0.15));
				}
				break;

			case 8:
				setFuel(0);
				break;
		}
	}

	public Set<EntityID> getVisibleFrom() {
		return visibleFrom;
	}

	// public void setVisibleFrom(Set<EntityID> visibleFrom) {
	// 	this.visibleFrom = visibleFrom;
	// 	this.visibleFrom.add(this.getId());
	// 	if (DebugHelper.DEBUG_MODE && !worldHelper.getScenarioInfo().getRawConfig().getBooleanValue(ConfigKey.KEY_PRECOMPUTE, false)) {
	// 		List<Integer> elementIds = Util.fetchIdValueFromElementIds(visibleFrom);
	// 		DebugHelper.VD_CLIENT.drawAsync(this.getId().getValue(), "VisibleFromAreas", (Serializable) elementIds);
	// 	}
	// }

	// public void setObservableAreas(List<EntityID> observableAreas) {
	// 	this.observableAreas = observableAreas;
	// 	if (DebugHelper.DEBUG_MODE && !worldHelper.getScenarioInfo().getRawConfig().getBooleanValue(ConfigKey.KEY_PRECOMPUTE, false)) {
	// 		List<Integer> elementIds = Util.fetchIdValueFromElementIds(observableAreas);
	// 		DebugHelper.VD_CLIENT.drawAsync(this.getId().getValue(), "ObservableAreas", (Serializable) elementIds);
	// 	}
	// }

	public void setLineOfSight(List<TIMRADLineOfSightPerception.TIMRADRay> lineOfSight) {
		this.lineOfSight = lineOfSight;
	}

	public List<TIMRADLineOfSightPerception.TIMRADRay> getLineOfSight() {
		return lineOfSight;
	}

	public double getHitRate() {
		return hitRate;
	}

	public void setHitRate(double hitRate) {
		this.hitRate = hitRate;
	}

	public int getLastSeenTime() {
		return lastSeenTime;
	}

	public void setLastSeenTime(int lastSeenTime) {
		this.lastSeenTime = lastSeenTime;
	}

	public int getLastUpdateTime() {
		return lastUpdateTime;
	}

	public void setLastUpdateTime(int lastUpdateTime) {
		this.lastUpdateTime = lastUpdateTime;
	}
}