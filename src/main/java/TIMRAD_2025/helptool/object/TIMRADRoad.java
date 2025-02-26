package TIMRAD_2025.helptool.object;

import TIMRAD_2025.helptool.TIMRADConstants;
import TIMRAD_2025.debugger.DebugHelper;
import TIMRAD_2025.helptool.ExpandApexes;
import TIMRAD_2025.helptool.Ruler;
import TIMRAD_2025.helptool.Util;
import TIMRAD_2025.helptool.TIMRADWorldHelper;
import TIMRAD_2025.helptool.GraphHelper;
import TIMRAD_2025.helptool.MyEdge;
import TIMRAD_2025.helptool.Node;
import adf.core.agent.info.AgentInfo;
import adf.core.launcher.ConfigKey;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;



import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.awt.geom.Area;
import java.io.Serializable;
import java.util.List;
import java.util.*;

public class TIMRADRoad {
	private double CLEAR_WIDTH; // 3m = 3000mm

	private Road selfRoad;
	private EntityID selfId;
	private TIMRADWorldHelper world;
	private GraphHelper graph;
	private AgentInfo agentInfo;

	private TIMRADLineOfSightPerception lineOfSightPerception;
	private List<EntityID> observableAreas;

	private List<TIMRADEdge> TIMRADEdges;
	private List<TIMRADBlockade> TIMRADBlockades = new ArrayList<>();

	private Pair<Line2D, Line2D> pfClearLines = null;
	private Area pfClearArea = null;

	private int lastUpdateTime = 0;
	private Polygon polygon;
	private int passablyLastResetTime = 0;
	private List<TIMRADLineOfSightPerception.TIMRADRay> lineOfSight;
	private Set<EntityID> visibleFrom;

	private Line2D roadCenterLine = null;

	private boolean isEntrance = false;
	private boolean isRoadCenterBlocked = false;
	private static final double COLLINEAR_THRESHOLD = 1.0E-3D;

	public TIMRADRoad(Road road, TIMRADWorldHelper world) {
		this.world = world;
		this.graph = world.getGraph();
		this.agentInfo = world.getAgentInfo();
		this.selfRoad = road;
		this.selfId = road.getID();
		this.lineOfSightPerception = new TIMRADLineOfSightPerception(world);
		this.TIMRADEdges = createTIMRADEdges();

		this.CLEAR_WIDTH = world.getConfig().repairRad;
		this.lineOfSight = new ArrayList<>();
		this.visibleFrom = new HashSet<>();
		createPolygon();
	}

	public TIMRADRoad(EntityID roadId, List<TIMRADEdge> edges) {
		this.selfId = roadId;
		this.TIMRADEdges = edges;
	}

	public void update() {
		lastUpdateTime = world.getTime();
		if (selfRoad.isBlockadesDefined()) {
			for (TIMRADEdge next : TIMRADEdges) {
				next.setOpenPart(next.getLine());
				next.setBlocked(false);
			}
			for (MyEdge myEdge : graph.getMyEdgesInArea(selfId)) {
				myEdge.setPassable(true);
			}
			this.TIMRADBlockades = createTIMRADBlockade();
			if (selfRoad.isBlockadesDefined()) {
				for (TIMRADEdge TIMRADEdge : TIMRADEdges) {
					if (TIMRADEdge.isPassable()) {
						TIMRADEdge.setOpenPart(TIMRADEdge.getLine());
						List<TIMRADBlockade> blockedStart = new ArrayList<>();
						List<TIMRADBlockade> blockedEnd = new ArrayList<>();
						for (TIMRADBlockade TIMRADBlockade : TIMRADBlockades) {

							if (Ruler.getDistance(TIMRADBlockade.getPolygon(), TIMRADEdge.getStart()) < TIMRADConstants.AGENT_PASSING_THRESHOLD_SMALL) {
								blockedStart.add(TIMRADBlockade);
							}
							if (Ruler.getDistance(TIMRADBlockade.getPolygon(), TIMRADEdge.getEnd()) < TIMRADConstants.AGENT_PASSING_THRESHOLD_SMALL) {
								blockedEnd.add(TIMRADBlockade);
							}
						}
						setTIMRADEdgeOpenPart(TIMRADEdge);
						if (TIMRADBlockade.size() == 1) {
							if (Util.containsEach(blockedEnd, blockedStart)) {
								TIMRADBlockades.get(0).addBlockedEdges(TIMRADEdge);
								TIMRADEdge.setBlocked(true);
							}
						} else {
							for (TIMRADBlockade block1 : blockedStart) {
								for (TIMRADBlockade block2 : blockedEnd) {
									if (Util.isPassable(block1.getPolygon(), block2.getPolygon(), TIMRADConstants.AGENT_PASSING_THRESHOLD_SMALL)) {
										TIMRADEdge.setBlocked(true);
										block1.addBlockedEdges(TIMRADEdge);
										block2.addBlockedEdges(TIMRADEdge);
									}

								}
							}
						}
					} else {
						for (TIMRADBlockade TIMRADBlockade : TIMRADBlockades) {
							double distance = Ruler.getDistance(TIMRADEdge.getLine(), TIMRADBlockade.getPolygon());

							if (distance < TIMRADConstants.AGENT_PASSING_THRESHOLD_SMALL) {
								TIMRADEdge.setBlocked(true);
								TIMRADBlockade.addBlockedEdges(TIMRADEdge);
							}

						}
					}
				}
			}
			updateNodePassably();
			updateMyEdgePassably();
		}
	}

	private boolean isTimeToResetPassably() {
		int resetTime = TIMRADConstants.ROAD_PASSABLY_RESET_TIME_IN_MEDIUM_MAP;
		return passablyLastResetTime <= lastUpdateTime && agentInfo.getTime() - passablyLastResetTime > resetTime &&
				agentInfo.getTime() - lastUpdateTime > resetTime;
	}
	
	public void resetPassably() {
		boolean isSeen = world.getRoadsSeen().contains(selfId);

		if (agentInfo.me() instanceof PoliceForce || passablyLastResetTime > lastUpdateTime) {
			return;
		}
		if (isTimeToResetPassably()) {
			reset();
		}
	}

	private void reset() {
		if (!(agentInfo.me() instanceof Human)) {
			return;
		}
		for (TIMRADEdge TIMRADEdge : TIMRADEdges) {
			TIMRADEdge.setBlocked(false);
			TIMRADEdge otherEdge = TIMRADEdge.getOtherSideEdge();
			TIMRADEdge.setOpenPart(TIMRADEdge.getLine());
			if (otherEdge != null) {
				TIMRADRoad TIMRADRoad = world.getTIMRADRoad(TIMRADEdge.getNeighbours().second());
				if (TIMRADRoad.getLastUpdateTime() < lastUpdateTime) {
					otherEdge.setOpenPart(otherEdge.getLine());
				}
			}
			rescuecore2.standard.entities.Area neighbour = (rescuecore2.standard.entities.Area) world.getEntity(TIMRADEdge.getNeighbours().second());
			if (TIMRADEdge.isPassable()) {
				Node node = graph.getNode((TIMRADEdge.getMiddlePoint()));
				if (node == null) {
					System.out.println("node == null in " + selfId);
					continue;
				}
				if (neighbour instanceof Road) {
					TIMRADRoad TIMRADRoad = world.getTIMRADRoad(neighbour.getID());
					TIMRADEdge neighbourEdge = TIMRADRoad.getTIMRADEdgeInPoint(TIMRADEdge.getMiddlePoint());
					if (neighbourEdge != null && !neighbourEdge.isBlocked()) {
						node.setPassable(true, agentInfo.getTime());
					}
				} else {
					node.setPassable(true, agentInfo.getTime());
				}
			}
		}
		for (MyEdge myEdge : graph.getMyEdgesInArea(selfId)) {
			myEdge.setPassable(true);
		}
		passablyLastResetTime = agentInfo.getTime();
	}

	private void updateNodePassably() {
		for (TIMRADEdge TIMRADEdge : TIMRADEdges) {
			if (TIMRADEdge.isPassable()) {
				Node node = graph.getNode(TIMRADEdge.getMiddlePoint());
				if (node == null) {
					continue;
				}
				if (TIMRADEdge.isBlocked() || TIMRADEdge.getOtherSideEdge().isBlocked()) {
					node.setPassable(false, agentInfo.getTime());
				} else {
					node.setPassable(true, agentInfo.getTime());
				}
			}
		}
	}

	private void updateMyEdgePassably() {
		for (int i = 0; i < TIMRADEdge.size() - 1; i++) {
			TIMRADEdge edge1 = TIMRADEdge.get(i);
			if (!edge1.isPassable()) {
				continue;
			}
			for (int j = i + 1; j < TIMRADEdge.size(); j++) {
				TIMRADEdge edge2 = TIMRADEdge.get(j);
				if (!edge2.isPassable()) {
					continue;
				}
				setMyEdgePassably(edge1, edge2, isPassable(edge1, edge2));
			}
		}
	}

	private void setMyEdgePassably(TIMRADEdge edge1, TIMRADEdge edge2, boolean passably) {
		if (!(agentInfo.me() instanceof Human) || !edge1.getNeighbours().second().equals(edge2.getNeighbours().second())) {
			return;
		}
		Node node1 = graph.getNode(edge1.getMiddlePoint());
		Node node2 = graph.getNode(edge2.getMiddlePoint());
		MyEdge myEdge = graph.getMyEdge(selfId, new Pair<>(node1, node2));
		if (myEdge != null) {
			myEdge.setPassable(passably);
		}
	}

	public boolean isPassable(TIMRADEdge from, TIMRADEdge to) {
		if (!from.getNeighbours().second().equals(to.getNeighbours().second())) {
			System.err.println("this 2 edge is not in a same area!!!");
			return false;
		}
		if (from.isBlocked() || to.isBlocked())
			return false;
		Pair<List<TIMRADEdge>, List<TIMRADEdge>> edgesBetween = getEdgesBetween(from, to, false);
	
		int count = TIMRADBlockade.size();
		List<TIMRADEdge> blockedEdges = new ArrayList<>();
		if (count == 1) {
			blockedEdges.addAll(TIMRADBlockade.get(0).getBlockedEdges());
		} else if (count > 1) {
			for (int i = 0; i < count - 1; i++) {
				TIMRADBlockade block1 = TIMRADBlockade.get(i);
				for (int j = i + 1; j < count; j++) {
					TIMRADBlockade block2 = TIMRADBlockade.get(j);
					if (isBlockedTwoSides(block1, edgesBetween)) {
						return false;
					}
					if (isBlockedTwoSides(block2, edgesBetween)) {
						return false;
					}
					if (isInSameSide(block1, block2, edgesBetween)) {
						continue;
					}
					if (Util.isPassable(block1.getPolygon(), block2.getPolygon(), TIMRADConstants.AGENT_PASSING_THRESHOLD)) {
						blockedEdges.removeAll(block1.getBlockedEdges());
						blockedEdges.addAll(block1.getBlockedEdges());
						blockedEdges.removeAll(block2.getBlockedEdges());
						blockedEdges.addAll(block2.getBlockedEdges());
					}
				}
			}
		} else if (count == 0) {
			return !(from.isBlocked() || to.isBlocked());
		}
		return !(Util.containsEach(blockedEdges, edgesBetween.first()) && Util.containsEach(blockedEdges, edgesBetween.second()));
	}

	private Pair<List<TIMRADEdge>, List<TIMRADEdge>> getEdgesBetween(TIMRADEdge edge1, TIMRADEdge edge2, boolean justImPassable) {
		List<TIMRADEdge> leftSideEdges = new ArrayList<>();
		List<TIMRADEdge> rightSideEdges = new ArrayList<>();
		Point2D startPoint1 = edge1.getStart();
		Point2D endPoint1 = edge1.getEnd();
		Point2D startPoint2 = edge2.getStart();
		Point2D endPoint2 = edge2.getEnd();

		boolean finishedLeft = false;
		boolean finishedRight = false;
		for (TIMRADEdge edge : TIMRADEdges) {
			if (finishedLeft && finishedRight)
				break;
			for (TIMRADEdge ed : TIMRADEdges) {
				if (finishedLeft && finishedRight)
					break;
				if (ed.equals(edge1) || ed.equals(edge2)) {
					continue;
				}
				if (startPoint1.equals(startPoint2) || startPoint1.equals(endPoint2)) {
					finishedLeft = true;
				}
				if (endPoint1.equals(startPoint2) || endPoint1.equals(endPoint2)) {
					finishedRight = true;
				}

				if (ed.getStart().equals(startPoint1) && !finishedLeft && !leftSideEdges.contains(ed)) {
					startPoint1 = ed.getEnd();
					if (!justImPassable || !ed.isPassable())
						leftSideEdges.add(ed);
					continue;
				}
				if (ed.getEnd().equals(startPoint1) && !finishedLeft && !leftSideEdges.contains(ed)) {
					startPoint1 = ed.getStart();
					if (!justImPassable || !ed.isPassable())
						leftSideEdges.add(ed);
					continue;
				}
				if (ed.getStart().equals(endPoint1) && !finishedRight && !rightSideEdges.contains(ed)) {
					endPoint1 = ed.getEnd();
					if (!justImPassable || !ed.isPassable())
						rightSideEdges.add(ed);
					continue;
				}
				if (ed.getEnd().equals(endPoint1) && !finishedRight && !rightSideEdges.contains(ed)) {
					endPoint1 = ed.getStart();
					if (!justImPassable || !ed.isPassable())
						rightSideEdges.add(ed);
					continue;
				}
			}
		}
		return new Pair<>(leftSideEdges, rightSideEdges);
	}

	private boolean isInSameSide(TIMRADBlockade block1, TIMRADBlockade block2, Pair<List<TIMRADEdge>, List<TIMRADEdge>> edgesBetween) {
		return edgesBetween.first().containsAll(block1.getBlockedEdges()) &&
				edgesBetween.first().containsAll(block2.getBlockedEdges()) ||
				edgesBetween.second().containsAll(block1.getBlockedEdges()) &&
						edgesBetween.second().containsAll(block2.getBlockedEdges());
	}

	private boolean isBlockedTwoSides(TIMRADBlockade block1, Pair<List<TIMRADEdge>, List<TIMRADEdge>> edgesBetween) {
		return Util.containsEach(edgesBetween.first(), block1.getBlockedEdges()) &&
				Util.containsEach(edgesBetween.second(), block1.getBlockedEdges());
	}

	private void createPolygon() {
		int[] apexList = selfRoad.getApexList();
		polygon = Util.getPolygon(apexList);
	}

	private List<TIMRADEdge> createTIMRADEdges() {
		List<TIMRADEdge> result = new ArrayList<>();

		for (Edge next : selfRoad.getEdges()) {
			result.add(new TIMRADEdge(world, next, selfRoad.getID()));
		}

		return result;
	}

	private List<TIMRADBlockade> createTIMRADBlockade() {
		List<TIMRADBlockade> result = new ArrayList<>();
		if (!selfRoad.isBlockadesDefined())
			return result;
		for (EntityID next : selfRoad.getBlockades()) {
			StandardEntity entity = world.getEntity(next, StandardEntity.class);
			if (entity == null)
				continue;
			if (!(entity instanceof Blockade))
				continue;
			Blockade bloc = (Blockade) entity;
			if (!bloc.isApexesDefined())
				continue;
			if (bloc.getApexes().length < 6)
				continue;
			result.add(new TIMRADBlockade(next, world));
		}

		return result;
	}

	private void setTIMRADEdgeOpenPart(TIMRADEdge edge) {
		List<Pair<Point2D, Point2D>> blockadePartPoints = new ArrayList<>();
		Point2D edgeStart = edge.getStart();
		Point2D edgeEnd = edge.getEnd();
		boolean isBlocked = false;
		List<TIMRADBlockade> totalBlockades = new ArrayList<>(TIMRADBlockades);
		TIMRADRoad neighborRoad = world.getTIMRADRoad(edge.getNeighbours().first());
		if (neighborRoad != null) {
			List<TIMRADBlockade> neighborBlockades = neighborRoad.getTIMRADBlockades();
			if (neighborBlockades != null) {
				totalBlockades.addAll(neighborBlockades);
			}
		}
		for (TIMRADBlockade blockade : totalBlockades) {
			boolean isStartBlocked = false;
			boolean isEndBlocked = false;
			if (blockade.getPolygon().contains(selfRoad.getX(), selfRoad.getY())) {
				isRoadCenterBlocked = true;
			}
			Polygon expand = Util.scaleBySize(blockade.getPolygon(), 10);
			if (expand.contains(edgeStart.getX(), edgeStart.getY())) {
				isStartBlocked = true;
			}
			if (expand.contains(edgeEnd.getX(), edgeEnd.getY())) {
				isEndBlocked = true;
			}

			Set<Point2D> intersections = Util.getIntersections(expand, edge.getLine());

			if (isStartBlocked && isEndBlocked) {
				isBlocked = true;
				blockadePartPoints.add(new Pair<>(edgeStart, edgeEnd));
				break;
			}else if (isStartBlocked) {
				double maxDistance = Double.MIN_VALUE, distance;
				Point2D blockadePartEnd = null;
				for (Point2D point : intersections) {
					distance = distance(point, edgeStart);
					if (distance > maxDistance) {
						maxDistance = distance;
						blockadePartEnd = point;
					}
				}
				if (blockadePartEnd!=null){
					blockadePartPoints.add(new Pair<>(edgeStart, blockadePartEnd));
				}
			} else if (isEndBlocked) {
				double maxDistance = Double.MIN_VALUE, distance;
				double mx = 1,haha;
				Point2D blockadePartStart = null;
				for (Point2D point : intersections) {
					distance = distance(point, edgeEnd);
					if (distance > maxDistance) {
						maxDistance = distance;
						blockadePartStart = point;
					}
				}
				if (blockadePartStart!=null){
					blockadePartPoints.add(new Pair<>(blockadePartStart, edgeEnd));
				}

			} else {
				if (!intersections.isEmpty() && intersections.size() > 1) {
					Pair<Point2D, Point2D> twoFarthestPoints = getTwoFarthestPoints(intersections);
					double distanceToFirst = Ruler.getDistance(edgeStart, twoFarthestPoints.first());
					double distanceToSecond = Ruler.getDistance(edgeStart, twoFarthestPoints.second());
					if (distanceToFirst < distanceToSecond) {
						blockadePartPoints.add(new Pair<>(twoFarthestPoints.first(), twoFarthestPoints.second()));
					} else {
						blockadePartPoints.add(new Pair<>(twoFarthestPoints.second(), twoFarthestPoints.first()));
					}
				}
			}
		}

		if (isBlocked) {
			edge.setBlocked(true);
			edge.setOpenPart(null);
		} else {
			List<Line2D> openPartLines = calcOpenPart(blockadePartPoints, edgeStart, edgeEnd);
			if (!openPartLines.isEmpty()) {
				edge.setOpenPart(openPartLines.get(openPartLines.size() - 1));
				if (Ruler.getLength(edge.getOpenPart()) <= TIMRADConstants.AGENT_MINIMUM_PASSING_THRESHOLD) {
					edge.setBlocked(true);
				}else {
					edge.setBlocked(false);
				}
			}
		}
	}

	private Pair<Point2D, Point2D> getTwoFarthestPoints(Set<Point2D> points) {
		double maxDistance = Double.MIN_VALUE;
		Point2D p1 = null;
		Point2D p2 = null;
		for (Point2D p3 : points) {
			for (Point2D p4 : points) {
				double distance = Ruler.getDistance(p3, p4);
				if (distance > maxDistance) {
					maxDistance = distance;
					p1 = p3;
					p2 = p4;
				}
			}
		}
		return new Pair<>(p1, p2);
	}

	private List<Line2D> calcOpenPart(List<Pair<Point2D, Point2D>> blockadePartPoints, Point2D edgeStart, Point2D edgeEnd) {
		blockadePartPoints.sort(new Util.DistanceComparator(edgeStart));
		blockadePartPoints.add(0, new Pair<>(null, edgeStart));
		blockadePartPoints.add(blockadePartPoints.size(), new Pair<>(edgeEnd, null));
		List<Line2D> openPartLines = new ArrayList<>();
		for (int i = 0; i < blockadePartPoints.size() - 1; i++) {
			if (Ruler.getDistance(blockadePartPoints.get(i).second(), edgeStart) < Ruler.getDistance(blockadePartPoints.get(i + 1).first(), edgeStart)) {
				openPartLines.add(new Line2D(blockadePartPoints.get(i).second(), blockadePartPoints.get(i + 1).first()));
			}

		}
		openPartLines.sort(new Util.LengthComparator());

		return openPartLines;
	}

	public Road getSelfRoad() {
		return selfRoad;
	}

	public EntityID getId() {
		return this.selfId;
	}

	public List<EntityID> getObservableAreas() {
		if (observableAreas == null || observableAreas.isEmpty()) {
			observableAreas = lineOfSightPerception.getVisibleAreas(getId());
		}
		return observableAreas;
	}

	public TIMRADEdge getTIMRADEdgeInPoint(Point2D middlePoint) {
		for (TIMRADEdge next : TIMRADEdges) {
			if (contains(next.getLine(), middlePoint, 1.0))
				return next;
		}

		return null;
	}

	public boolean isNeedlessToClear() {
		double buildingEntranceLength = 0.0;
		double maxUnpassableEdgeLength = Double.MIN_VALUE;
		double length;

		Edge buildingEntrance = null;

		for (Edge next : selfRoad.getEdges()) {
			if (next.isPassable()) {
				StandardEntity entity = world.getEntity(next.getNeighbour(), StandardEntity.class);
				if (entity instanceof Building) {
					buildingEntranceLength = distance(next.getStart(), next.getEnd());
					buildingEntrance = next;
				}
			} else {
				length = distance(next.getStart(), next.getEnd());
				if (length > maxUnpassableEdgeLength) {
					maxUnpassableEdgeLength = length;
				}
			}
		}

		if (buildingEntrance == null)
			return true;
		double rad = buildingEntranceLength + maxUnpassableEdgeLength;
		Area entranceArea = entranceArea(buildingEntrance.getLine(), rad);

		Set<EntityID> blockadeIds = new HashSet<>();

		if (selfRoad.isBlockadesDefined()) {
			blockadeIds.addAll(selfRoad.getBlockades());
		}

		for (EntityID next : selfRoad.getNeighbours()) {
			StandardEntity entity = world.getEntity(next, StandardEntity.class);
			if (entity instanceof Road) {
				Road road = (Road) entity;
				if (road.isBlockadesDefined())
					blockadeIds.addAll(road.getBlockades());
			}
		}

		for (EntityID next : blockadeIds) {
			StandardEntity entity = world.getEntity(next, StandardEntity.class);
			if (entity == null)
				continue;
			if (!(entity instanceof Blockade))
				continue;
			Blockade blockade = (Blockade)entity;
			if (!blockade.isApexesDefined())
				continue;
			//？？
			if (blockade.getApexes().length < 6)
				continue;
			Polygon po = Util.getPolygon(blockade.getApexes());
			Area blocArea = new Area(po);
			blocArea.intersect(entranceArea);
			//??
			if (!blocArea.getPathIterator(null).isDone())
				return false;
		}
		return true;
	}

	private Area entranceArea(Line2D line, double rad) {
		double theta = Math.atan2(line.getEndPoint().getY() - line.getOrigin().getY(),
				line.getEndPoint().getX() - line.getOrigin().getX());
		theta = theta - Math.PI / 2;
		while (theta > Math.PI || theta < -Math.PI) {
			if (theta > Math.PI)
				theta -= 2 * Math.PI;
			else
				theta += 2 * Math.PI;
		}
		int x = (int)(rad * Math.cos(theta)), y = (int)(rad * Math.sin(theta));

		Polygon polygon = new Polygon();
		polygon.addPoint((int)(line.getOrigin().getX() + x), (int)(line.getOrigin().getY() + y));
		polygon.addPoint((int)(line.getEndPoint().getX() + x), (int)(line.getEndPoint().getY() + y));
		polygon.addPoint((int)(line.getEndPoint().getX() - x), (int)(line.getEndPoint().getY() - y));
		polygon.addPoint((int)(line.getOrigin().getX() - x), (int)(line.getOrigin().getY() - y));

		return new Area(polygon);
	}

	public List<TIMRADEdge> getTIMRADEdgesTo(EntityID neighbourId) {
		List<TIMRADEdge> result = new ArrayList<>();

		for (TIMRADEdge next : TIMRADEdges) {
			if (next.isPassable() && next.getNeighbours().first().equals(neighbourId)) {
				result.add(next);
			}
		}

		return result;
	}

	public Set<TIMRADEdge> getPassableEdges() {
		Set<TIMRADEdge> result = new HashSet<>();

		for (TIMRADEdge next : TIMRADEdges) {
			if (next.isPassable() && !next.isBlocked()) {
				result.add(next);
			}
		}

		return result;
	}

	public boolean isRoadCenterBlocked() {
		return this.isRoadCenterBlocked;
	}

	public boolean isPassable() {
		if (isAllEdgePassable() || isOneEdgeUnpassable()) {

			return getPassableEdges().size() > 1;
		} else {
			List<TIMRADBlockade> blockades = new LinkedList<>(getTIMRADBlockades());
			
			for (TIMRADEscapePoint next : getEscapePoint(this, 500)) {
				blockades.removeAll(next.getRelateBlockade());
			}
			
			if (blockades.isEmpty())
				return true;
			return false;
		}
	}

	public boolean isPassableForPF() {
		if (isAllEdgePassable() || isOneEdgeUnpassable()) {
			return getPassableEdges().size() > 1;
		}
		boolean isPassable = true;
		List<Polygon> blockadePolygons = getBlockadePolygons(10);
		for (Polygon polygon : blockadePolygons) {
			for (TIMRADEdge edge : TIMRADEdges) {
				TIMRADEdge oppositeEdge = getOppositeEdge(edge);
				if (Util.hasIntersectLine(polygon, Util.improveLineBothSides(edge.getLine(), 300000)) &&
						Util.hasIntersectLine(polygon, Util.improveLineBothSides(oppositeEdge.getLine(), 300000))) {
					isPassable = false;
					break;
				}
			}
			if (!isPassable) {
				break;
			}
		}
		List<TIMRADBlockade> blockades = new LinkedList<>(getTIMRADBlockades());

		for (TIMRADEscapePoint next : getEscapePoint(this, 500)) {
			blockades.removeAll(next.getRelateBlockade());
		}

		return blockades.isEmpty();
	}

	public boolean isEntrancePassable() {
		return false;
	}

	public boolean isAllEdgePassable() {
		for (TIMRADEdge next : TIMRADEdges) {
			if (!next.isPassable())
				return false;
		}
		return true;
	}

	public boolean isOneEdgeUnpassable() {
		int count = 0;
		for (TIMRADEdge next : TIMRADEdges) {
			if (!next.isPassable())
				count++;
		}

		if (count == 1)
			return true;
		else
			return false;
	}

	public boolean isEntrance() {
		return this.isEntrance;
	}

	public boolean isEntranceNeighbour() {
		return false;
	}

	public void setEntrance(boolean entrance) {
		this.isEntrance = entrance;
	}

	public Pair<Line2D, Line2D> getPfClearLine(TIMRADRoad road) {

		if (this.pfClearLines != null)
			return this.pfClearLines;

		if (road.getTIMRADEdges().size() != 4)
			return null;
		if (road.isAllEdgePassable())
			return null;

			TIMRADEdge edge_1 = road.getTIMRADEdges().get(0);
			TIMRADEdge edge_2 = road.getTIMRADEdges().get(1);
			TIMRADEdge edge_3 = road.getTIMRADEdges().get(2);
			TIMRADEdge edge_4 = road.getTIMRADEdges().get(3);

		Line2D line_1 = null, line_2 = null, line_3 = null, line_4 = null;

		if (edge_1.isPassable() && edge_3.isPassable()) {
			roadCenterLine = new Line2D(edge_1.getMiddlePoint(), edge_3.getMiddlePoint());

			Point2D perpendicular_1, perpendicular_2;

			Pair<Double, Boolean> dis = ptSegDistSq(edge_2.getLine(), edge_1.getStart());
			if (dis.second().booleanValue()) { // the point is out the range of this line
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_4.getLine(), edge_1.getEnd());
				line_1 = new Line2D(perpendicular_1, edge_1.getEnd());
			} else { // the point is within the range of this line
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_2.getLine(), edge_1.getStart());
				line_1 = new Line2D(edge_1.getStart(), perpendicular_1);
			}

			dis = ptSegDistSq(edge_4.getLine(), edge_3.getStart());
			if (dis.second().booleanValue()) {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_2.getLine(), edge_3.getEnd());
				line_2 = new Line2D(edge_3.getEnd(), perpendicular_2);
			} else {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_4.getLine(), edge_3.getStart());
				line_2 = new Line2D(perpendicular_2, edge_3.getStart());
			}
		} else if (edge_2.isPassable() && edge_4.isPassable()) {
			roadCenterLine = new Line2D(edge_2.getMiddlePoint(), edge_4.getMiddlePoint());

			Point2D perpendicular_1, perpendicular_2;

			Pair<Double, Boolean> dis = ptSegDistSq(edge_3.getLine(), edge_2.getStart());
			if (dis.second().booleanValue()) {
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_1.getLine(), edge_2.getEnd());
				line_1 = new Line2D(perpendicular_1, edge_2.getEnd());
			} else {
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_3.getLine(), edge_2.getStart());
				line_1 = new Line2D(edge_2.getStart(), perpendicular_1);
			}

			dis = ptSegDistSq(edge_1.getLine(), edge_4.getStart());
			if (dis.second().booleanValue()) {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_3.getLine(), edge_4.getEnd());
				line_2 = new Line2D(edge_4.getEnd(), perpendicular_2);
			} else {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_1.getLine(), edge_4.getStart());
				line_2 = new Line2D(perpendicular_2, edge_4.getStart());
			}
		}

		double rate_1 = CLEAR_WIDTH / getLength(line_1);
		double rate_2 = CLEAR_WIDTH / getLength(line_2);
		Point2D mid_1 = getMiddle(line_1), mid_2 = getMiddle(line_2);

		Point2D end_1 = (new Line2D(mid_1, line_1.getOrigin())).getPoint(rate_1);
		Point2D end_2 = (new Line2D(mid_2, line_2.getOrigin())).getPoint(rate_2);
		line_3 = new Line2D(end_1, end_2);

		end_1 = (new Line2D(mid_1, line_1.getEndPoint())).getPoint(rate_1);
		end_2 = (new Line2D(mid_2, line_2.getEndPoint())).getPoint(rate_2);
		line_4 = new Line2D(end_1, end_2);

		this.pfClearLines = new Pair<Line2D, Line2D>(line_3, line_4);
		return this.pfClearLines;
	}

	public Area getPfClearArea(TIMRADRoad road) {

		if (this.pfClearArea != null)
			return pfClearArea;

		if (road.getTIMRADEdges().size() != 4)
			return null;
		if (road.isAllEdgePassable())
			return null;

		TIMRADEdge edge_1 = road.getTIMRADEdges().get(0);
		TIMRADEdge edge_2 = road.getTIMRADEdges().get(1);
		TIMRADEdge edge_3 = road.getTIMRADEdges().get(2);
		TIMRADEdge edge_4 = road.getTIMRADEdges().get(3);

		Polygon area = new Polygon();

		Line2D line_1 = null, line_2 = null;

		if (edge_1.isPassable() && edge_3.isPassable()) {
			roadCenterLine = new Line2D(edge_1.getMiddlePoint(), edge_3.getMiddlePoint());
			Point2D perpendicular_1, perpendicular_2;

			Pair<Double, Boolean> dis = ptSegDistSq(edge_2.getLine(), edge_1.getStart());
			if (!dis.second().booleanValue()) { // the point is out the range of this line
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_4.getLine(), edge_1.getEnd());
				line_1 = new Line2D(perpendicular_1, edge_1.getEnd());
			} else { // the point is within the range of this line
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_2.getLine(), edge_1.getStart());
				line_1 = new Line2D(edge_1.getStart(), perpendicular_1);
			}

			dis = ptSegDistSq(edge_4.getLine(), edge_3.getStart());
			if (!dis.second().booleanValue()) {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_2.getLine(), edge_3.getEnd());
				line_2 = new Line2D(edge_3.getEnd(), perpendicular_2);
			} else {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_4.getLine(), edge_3.getStart());
				line_2 = new Line2D(perpendicular_2, edge_3.getStart());
			}
		} else if (edge_2.isPassable() && edge_4.isPassable()) {
			roadCenterLine = new Line2D(edge_2.getMiddlePoint(), edge_4.getMiddlePoint());
			Point2D perpendicular_1, perpendicular_2;

			Pair<Double, Boolean> dis = ptSegDistSq(edge_3.getLine(), edge_2.getStart());
			if (!dis.second().booleanValue()) {
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_1.getLine(), edge_2.getEnd());
				line_1 = new Line2D(perpendicular_1, edge_2.getEnd());
			} else {
				perpendicular_1 = GeometryTools2D.getClosestPoint(edge_3.getLine(), edge_2.getStart());
				line_1 = new Line2D(edge_2.getStart(), perpendicular_1);
			}

			dis = ptSegDistSq(edge_1.getLine(), edge_4.getStart());
			if (!dis.second().booleanValue()) {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_3.getLine(), edge_4.getEnd());
				line_2 = new Line2D(edge_4.getEnd(), perpendicular_2);
			} else {
				perpendicular_2 = GeometryTools2D.getClosestPoint(edge_1.getLine(), edge_4.getStart());
				line_2 = new Line2D(perpendicular_2, edge_4.getStart());
			}
		}

		double rate_1 = CLEAR_WIDTH / getLength(line_1);
		double rate_2 = CLEAR_WIDTH / getLength(line_2);
		Point2D mid_1 = getMiddle(line_1), mid_2 = getMiddle(line_2);

		Point2D end_1 = (new Line2D(mid_1, line_1.getOrigin())).getPoint(rate_1);
		Point2D end_2 = (new Line2D(mid_2, line_2.getOrigin())).getPoint(rate_2);
		area.addPoint((int)end_1.getX(), (int)end_1.getY());
		area.addPoint((int)end_2.getX(), (int)end_2.getY());

		end_1 = (new Line2D(mid_1, line_1.getEndPoint())).getPoint(rate_1);
		end_2 = (new Line2D(mid_2, line_2.getEndPoint())).getPoint(rate_2);

		// the order of the following two lines should not be change
		area.addPoint((int)end_2.getX(), (int)end_2.getY());
		area.addPoint((int)end_1.getX(), (int)end_1.getY());

		this.pfClearArea = new Area(area);
		return this.pfClearArea;
	}

	public Line2D getRoadCenterLine() {
		return this.roadCenterLine;
	}

	private boolean contains(Line2D line, Point2D point, double threshold) {

		double pos = java.awt.geom.Line2D.ptSegDist(line.getOrigin().getX(), line.getOrigin().getY(),
				line.getEndPoint().getX(), line.getEndPoint().getY(), point.getX(), point.getY());
		if (pos <= threshold)
			return true;

		return false;
	}

	private double distance(Point2D first, Point2D second) {
		return Math.hypot(first.getX() - second.getX(), first.getY() - second.getY());
	}

	public List<TIMRADEscapePoint> getEscapePoint(TIMRADRoad road, int threshold) {
		List<TIMRADEscapePoint> m_p_points = new ArrayList<>();

		for (TIMRADBlockade next : road.getTIMRADBlockades()) {
			if (next == null)
				continue;
			Polygon expan = next.getPolygon();

			for(TIMRADEdge TIMRADEdge : road.getTIMRADEdges()) {
				TIMRADEscapePoint p = findPoints(TIMRADEdge, expan, next);
				if (p == null) {
					continue;
				} else {
					m_p_points.add(p);
				}
			}
		}

		filter(road, m_p_points, threshold);
		return m_p_points;
	}

	private TIMRADEscapePoint findPoints(TIMRADEdge TIMRADEdge, Polygon expan, TIMRADBlockade next) {
		if (TIMRADEdge.isPassable()) {
			// do nothing
		} else {
			if (hasIntersection(expan, TIMRADEdge.getLine())) {
				return null;
			}
			double minDistance = Double.MAX_VALUE, distance;
			Pair<Integer, Integer> minDistanceVertex = null;

			for (Pair<Integer, Integer> vertex : next.getVertexesList()) {

				Pair<Double, Boolean> dis = ptSegDistSq(TIMRADEdge.getStart().getX(),
				TIMRADEdge.getStart().getY(), TIMRADEdge.getEnd().getX(),
				TIMRADEdge.getEnd().getY(), vertex.first(), vertex.second());

				if (dis.second().booleanValue())
					continue;
				distance = dis.first().doubleValue();

				if (distance < minDistance) {
					minDistance = distance;
					minDistanceVertex = vertex;
				}
			}

			if (minDistanceVertex == null)
				return null;

			Point2D perpendicular = GeometryTools2D.getClosestPoint(TIMRADEdge.getLine(),
					new Point2D(minDistanceVertex.first(), minDistanceVertex.second()));

			Point middlePoint = getMiddle(minDistanceVertex, perpendicular);

			Point2D vertex = new Point2D(minDistanceVertex.first(), minDistanceVertex.second());
			Point2D perpenPoint = new Point2D(perpendicular.getX(), perpendicular.getY());

			Line2D lin = new Line2D(vertex, perpenPoint);

			return new TIMRADEscapePoint(middlePoint, lin, next);
		}

		return null;
	}

	private void filter(TIMRADRoad road, List<TIMRADEscapePoint> m_p_points, int threshold) {
		Mark:for (Iterator<TIMRADEscapePoint> itor = m_p_points.iterator(); itor.hasNext(); ) {

			TIMRADEscapePoint m_p = itor.next();
			for (TIMRADEdge edge : road.getTIMRADEdges()) {
				if (edge.isPassable())
					continue;
				if (contains(edge.getLine(), m_p.getUnderlyingPoint(), threshold / 2)) {
					itor.remove();
					continue Mark;
				}
			}

			for (TIMRADBlockade blockade : road.getTIMRADBlockades()) {
				if (blockade == null)
					continue;
				Polygon polygon = blockade.getPolygon();
				Polygon po = ExpandApexes.expandApexes(blockade.getSelfBlockade(), 200);


				if (po.contains(m_p.getLine().getEndPoint().getX(), m_p.getLine().getEndPoint().getY())) {

					Set<Point2D> intersections = Util.getIntersections(polygon, m_p.getLine());

					double minDistance = Double.MAX_VALUE, distance;
					Point2D closest = null;
					boolean shouldRemove = false;
					for (Point2D inter : intersections) {
						distance = Ruler.getDistance(m_p.getLine().getOrigin(), inter);

						if (distance > threshold && distance < minDistance) {
							minDistance = distance;
							closest = inter;
						}
						shouldRemove = true;
					}

					if (closest != null) {
						Point p = getMiddle(m_p.getLine().getOrigin(), closest);
						m_p.getUnderlyingPoint().setLocation(p);
						m_p.addTIMRADBlockade(blockade);
					} else if (shouldRemove){
						itor.remove();
						continue Mark;
					}
				}

				if (po.contains(m_p.getUnderlyingPoint())) {
					itor.remove();
					continue Mark;
				}
			}
		}
	}

	private boolean contains(Line2D line, Point point, double threshold) {

		double pos = java.awt.geom.Line2D.ptSegDist(line.getOrigin().getX(),
				line.getOrigin().getY(), line.getEndPoint().getX(), line
						.getEndPoint().getY(), point.getX(), point.getY());
		if (pos <= threshold)
			return true;

		return false;
	}

	private Pair<Double, Boolean> ptSegDistSq(Line2D line, Point2D point) {
		return ptSegDistSq((int)line.getOrigin().getX(), (int)line.getOrigin().getY(),
				(int)line.getEndPoint().getX(), (int)line.getEndPoint().getY(),
				(int)point.getX(), (int)point.getY());
	}

	private Pair<Double, Boolean> ptSegDistSq(double x1, double y1, double x2,
                                              double y2, double px, double py) {

		x2 -= x1;
		y2 -= y1;

		px -= x1;
		py -= y1;

		double dotprod = px * x2 + py * y2;

		double projlenSq;

		if (dotprod <= 0) {
			projlenSq = 0;
		} else {
			px = x2 - px;
			py = y2 - py;
			dotprod = px * x2 + py * y2;

			if (dotprod <= 0.0) {
				projlenSq = 0.0;
			} else {
				projlenSq = dotprod * dotprod / (x2 * x2 + y2 * y2);
			}
		}

		double lenSq = px * px + py * py - projlenSq;

		if (lenSq < 0)
			lenSq = 0;

		if (projlenSq == 0) {
			// the target point out of this line
			return new Pair<Double, Boolean>(Math.sqrt(lenSq), true);
		} else {
			// the target point within this line
			return new Pair<Double, Boolean>(Math.sqrt(lenSq), false);
		}
	}

	public boolean hasIntersection(Polygon polygon, Line2D line) {
		List<Line2D> polyLines = getLines(polygon);
		for (Line2D ln : polyLines) {

			math.geom2d.line.Line2D line_1 = new math.geom2d.line.Line2D(
					line.getOrigin().getX(), line.getOrigin().getY(),
					line.getEndPoint().getX(), line.getEndPoint().getY());

			math.geom2d.line.Line2D line_2 = new math.geom2d.line.Line2D(
					ln.getOrigin().getX(), ln.getOrigin().getY(),
					ln.getOrigin().getX(), ln.getOrigin().getY());

			if (math.geom2d.line.Line2D.intersects(line_1, line_2)) {

				return true;
			}
		}
		return false;
	}

	private List<Line2D> getLines(Polygon polygon) {
		List<Line2D> lines = new ArrayList<>();
		int count = polygon.npoints;
		for (int i = 0; i < count; i++) {
			int j = (i + 1) % count;
			Point2D p1 = new Point2D(polygon.xpoints[i], polygon.ypoints[i]);
			Point2D p2 = new Point2D(polygon.xpoints[j], polygon.ypoints[j]);
			Line2D line = new Line2D(p1, p2);
			lines.add(line);
		}
		return lines;
	}
	
	private Point getMiddle(Pair<Integer, Integer> first, Point2D second) {
		int x = first.first() + (int)second.getX();
		int y = first.second() + (int)second.getY();

		return new Point(x / 2, y / 2);
	}

	private Point getMiddle(Point2D first, Point2D second) {
		int x = (int)(first.getX() + second.getX());
		int y = (int)(first.getY() + second.getY());

		return new Point(x / 2, y / 2);
	}

	private Point2D getMiddle(Line2D line) {
		double x = line.getOrigin().getX() + line.getEndPoint().getX();
		double y = line.getOrigin().getY() + line.getEndPoint().getY();

		return new Point2D(x / 2, y / 2);
	}

	private int getLength(Line2D line) {
		return (int) Ruler.getDistance(line.getOrigin(), line.getEndPoint());
	}

	public int getLastUpdateTime() {
		return lastUpdateTime;
	}

	public Polygon getPolygon() {
		return polygon;
	}

	public TIMRADEdge getOppositeEdge(TIMRADEdge edge) {
		if (!TIMRADEdge.contains(edge)) {
			return null;
		}
		List<Pair<TIMRADEdge, Line2D>> edgeLinesExcept = getEdgeLinesExcept(edge);
		edgeLinesExcept.sort(new Util.AngleComparator(edge.getLine()));
		return !edgeLinesExcept.isEmpty() ? edgeLinesExcept.get(0).first() : null;
	}

	public TIMRADEdge getOppositePassableEdge(TIMRADEdge edge) {
		if (!TIMRADEdge.contains(edge)) {
			return null;
		}
		List<Pair<TIMRADEdge, Line2D>> passableEdgeLinesExcept = getPassableEdgeLinesExcept(edge);
		passableEdgeLinesExcept.sort(new Util.AngleComparator(edge.getLine()));
		return !passableEdgeLinesExcept.isEmpty() ? passableEdgeLinesExcept.get(0).first() : null;
	}

	public List<Pair<TIMRADEdge, Line2D>> getPassableEdgeLines() {
		List<Pair<TIMRADEdge, Line2D>> result = new ArrayList<>();
		for (TIMRADEdge edge : TIMRADEdges) {
			if (edge.isPassable()) {
				result.add(new Pair<>(edge, edge.getLine()));
			}
		}
		return result;
	}

	public List<Pair<TIMRADEdge, Line2D>> getPassableEdgeLinesExcept(TIMRADEdge exceptEdge) {
		List<Pair<TIMRADEdge, Line2D>> result = new ArrayList<>();
		math.geom2d.line.Line2D exceptLine = Util.convertLine(exceptEdge.getLine());
		for (TIMRADEdge edge : TIMRADEdges) {
			math.geom2d.line.Line2D line = Util.convertLine(edge.getLine());
			if (edge.isPassable() && Util.isCollinear(exceptLine, line, COLLINEAR_THRESHOLD)) {
				result.add(new Pair<>(edge, edge.getLine()));
			}
		}
		return result;
	}

	public List<Pair<TIMRADEdge, Line2D>> getEdgeLinesExcept(TIMRADEdge exceptEdge) {
		List<Pair<TIMRADEdge, Line2D>> result = new ArrayList<>();
		math.geom2d.line.Line2D exceptLine = Util.convertLine(exceptEdge.getLine());
		for (TIMRADEdge edge : TIMRADEdges) {
			math.geom2d.line.Line2D line = Util.convertLine(edge.getLine());
			if (!Util.isCollinear(exceptLine, line, COLLINEAR_THRESHOLD)) {
				result.add(new Pair<>(edge, edge.getLine()));
			}
		}
		return result;
	}

	public TIMRADRoad getOppositePassableEdgeRoad(TIMRADEdge edge) {
		TIMRADEdge oppositeEdge = getOppositePassableEdge(edge);
		EntityID id = oppositeEdge.getNeighbours().first();
		return world.getTIMRADRoad(id);
	}

	public List<Polygon> getBlockadePolygons() {
		List<Polygon> result = new ArrayList<>();
		for (TIMRADBlockade blockade : TIMRADBlockades) {
			result.add(blockade.getPolygon());
		}
		return result;
	}

	public List<Polygon> getBlockadePolygons(int scale) {
		List<Polygon> result = new ArrayList<>();
		for (TIMRADBlockade blockade : TIMRADBlockades) {
			Polygon polygon = Util.scaleBySize(blockade.getPolygon(), scale);
			result.add(polygon);
		}
		return result;
	}


	public void setTIMRADBlockades(List<TIMRADBlockade> blockades) {
		this.TIMRADBlockades.clear();
		this.TIMRADBlockades.addAll(blockades);
	}

	public List<TIMRADEdge> getTIMRADEdges() {
		return this.TIMRADEdges;
	}

	public List<TIMRADBlockade> getTIMRADBlockades() {
		return this.TIMRADBlockades;
	}

	@SuppressWarnings("unused")
	public void setObservableAreas(List<EntityID> observableAreas) {
		this.observableAreas = observableAreas;
		if (DebugHelper.DEBUG_MODE && !world.getScenarioInfo().getRawConfig().getBooleanValue(ConfigKey.KEY_PRECOMPUTE, false)) {
			List<Integer> elementIds = Util.fetchIdValueFromElementIds(observableAreas);
			DebugHelper.VD_CLIENT.drawAsync(this.getId().getValue(), "ObservableAreas", (Serializable) elementIds);
		}
	}

	public void setLineOfSight(List<TIMRADLineOfSightPerception.TIMRADRay> rays) {
		this.lineOfSight = rays;
	}

	public List<TIMRADLineOfSightPerception.TIMRADRay> getLineOfSight() {
		return lineOfSight;
	}

	public Set<EntityID> getVisibleFrom() {
		return visibleFrom;
	}

	@SuppressWarnings("unused")
	public void setVisibleFrom(Set<EntityID> visibleFrom) {
		this.visibleFrom = visibleFrom;
		if (DebugHelper.DEBUG_MODE && !world.getScenarioInfo().getRawConfig().getBooleanValue(ConfigKey.KEY_PRECOMPUTE, false)) {
			List<Integer> elementIds = Util.fetchIdValueFromElementIds(visibleFrom);
			DebugHelper.VD_CLIENT.drawAsync(this.getId().getValue(), "VisibleFromAreas", (Serializable) elementIds);
		}
	}

}

