package TIMRAD_2025.helptool;

import TIMRAD_2025.helptool.ExpandApexes;
import TIMRAD_2025.helptool.Util;
import TIMRAD_2025.helptool.TIMRADEdge;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;
import TIMRAD_2025.helptool.object.TIMRADLineOfSightPerception;
import TIMRAD_2025.helptool.TIMRADEscapePointHelper;


import java.awt.*;
import java.awt.geom.Area;
import java.util.List;
import java.util.*;


public class TIMRADRoadHelper {
	private double CLEAR_WIDTH; // 3m = 3000mm

	private Road selfRoad;
	private EntityID selfId;
	private WorldInfo world;

	private TIMRADLineOfSightPerception lineOfSightPerception;
	private List<EntityID> observableAreas;

	private List<TIMRADEdgeHelper> TIMRADEdges;
	private List<TIMRADBlockadeHelper> TIMRADBlockades = new LinkedList<>();

	private Pair<Line2D, Line2D> pfClearLines = null;
	private Area pfClearArea = null;

	private Line2D roadCenterLine = null;

	private boolean isEntrance = false;
	private boolean isRoadCenterBlocked = false;

	public TIMRADRoadHelper(Road road, WorldInfo world, ScenarioInfo scenarioInfo) {
		this.world = world;
		this.selfRoad = road;
		this.selfId = road.getID();
		this.lineOfSightPerception = new TIMRADLineOfSightPerception(world);
		this.TIMRADEdges = createTIMRADEdges();

		this.CLEAR_WIDTH = scenarioInfo.getClearRepairRad();
	}

	public TIMRADRoadHelper(EntityID roadId, List<TIMRADEdgeHelper> edges) {
		this.selfId = roadId;
		this.TIMRADEdges = edges;
	}

	public void update() {
		for (TIMRADEdgeHelper next : TIMRADEdges) {
			next.setOpenPart(next.getLine());
			next.setBlocked(false);
		}

		this.TIMRADBlockades.clear();

		if (selfRoad.isBlockadesDefined()) {
			for (TIMRADEdgeHelper next : TIMRADEdges) {
				if (!next.isPassable())
					continue;
				// TODO July 9, 2014 Time: 2:57pm
				setTIMRADEdgeOpenPart(next);
			}

			this.TIMRADBlockades = createTIMRADBlockade();
		}
	}

	private List<TIMRADEdgeHelper> createTIMRADEdges() {
		List<TIMRADEdgeHelper> result = new LinkedList<>();

		for (Edge next : selfRoad.getEdges()) {
			result.add(new TIMRADEdgeHelper(world, next, selfRoad.getID()));
		}

		return result;
	}

	private List<TIMRADBlockadeHelper> createTIMRADBlockade() {
		List<TIMRADBlockadeHelper> result = new LinkedList<>();
		if (!selfRoad.isBlockadesDefined())
			return result;
		for (EntityID next : selfRoad.getBlockades()) {
			StandardEntity entity = world.getEntity(next);
			if (entity == null)
				continue;
			if (!(entity instanceof Blockade))
				continue;
			Blockade bloc = (Blockade) entity;
			if (!bloc.isApexesDefined())
				continue;
			if (bloc.getApexes().length < 6)
				continue;
			result.add(new TIMRADBlockadeHelper(next, world));
		}

		return result;
	}

	private void setTIMRADEdgeOpenPart(TIMRADEdgeHelper edge) {
		Polygon expand = null;
		boolean isStartBlocked = false, isEndBlocked = false;

		Point2D openPartStart = null, openPartEnd = null;

		for (TIMRADBlockadeHelper next : TIMRADBlockades) {
			if (next.getPolygon().contains(selfRoad.getX(), selfRoad.getY()))
				isRoadCenterBlocked = true;

			expand = ExpandApexes.expandApexes(next.getSelfBlockade(), 10);

			if (expand.contains(edge.getStart().getX(), edge.getStart().getY())) {
				isStartBlocked = true;
			} else if (expand.contains(edge.getEnd().getX(), edge.getEnd().getY())) {
				isEndBlocked = true;
			}

			if (isStartBlocked && isEndBlocked)
				continue;

			Set<Point2D> intersections = Util.getIntersections(expand, edge.getLine());

			if (isStartBlocked) {
				double minDistance = Double.MAX_VALUE, distance;
				openPartEnd = edge.getEnd();
				for (Point2D point : intersections) {
					distance = distance(point, openPartEnd);
					if (distance < minDistance) {
						minDistance = distance;
						openPartStart = point;
					}
				}
			} else if (isEndBlocked) {
				double minDistance = Double.MAX_VALUE, distance;
				openPartStart = edge.getStart();
				for (Point2D point : intersections) {
					distance = distance(point, openPartStart);
					if (distance < minDistance) {
						minDistance = distance;
						openPartEnd = point;
					}
				}
			}

			if (openPartStart == null || openPartEnd == null || distance(openPartStart, openPartEnd) < 200) {
				edge.setBlocked(true);
				edge.setOpenPart(null);
				break;
			} else {
				edge.setOpenPart(openPartStart, openPartEnd);
			}
		}
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

	public TIMRADEdgeHelper getTIMRADEdgeInPoint(Point2D middlePoint) {
		for (TIMRADEdgeHelper next : TIMRADEdges) {
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
				StandardEntity entity = world.getEntity(next.getNeighbour());
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
			StandardEntity entity = world.getEntity(next);
			if (entity instanceof Road) {
				Road road = (Road) entity;
				if (road.isBlockadesDefined())
					blockadeIds.addAll(road.getBlockades());
			}
		}

		for (EntityID next : blockadeIds) {
			StandardEntity entity = world.getEntity(next);
			if (entity == null)
				continue;
			if (!(entity instanceof Blockade))
				continue;
			Blockade blockade = (Blockade) entity;
			if (!blockade.isApexesDefined())
				continue;
			if (blockade.getApexes().length < 6)
				continue;
			Polygon po = Util.getPolygon(blockade.getApexes());
			Area blocArea = new Area(po);
			blocArea.intersect(entranceArea);
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
		int x = (int) (rad * Math.cos(theta)), y = (int) (rad * Math.sin(theta));

		Polygon polygon = new Polygon();
		polygon.addPoint((int) (line.getOrigin().getX() + x), (int) (line.getOrigin().getY() + y));
		polygon.addPoint((int) (line.getEndPoint().getX() + x), (int) (line.getEndPoint().getY() + y));
		polygon.addPoint((int) (line.getEndPoint().getX() - x), (int) (line.getEndPoint().getY() - y));
		polygon.addPoint((int) (line.getOrigin().getX() - x), (int) (line.getOrigin().getY() - y));

		return new Area(polygon);
	}

	public List<TIMRADEdgeHelper> getTIMRADEdgeTo(EntityID neighbourId) {
		List<TIMRADEdgeHelper> result = new LinkedList<>();

		for (TIMRADEdgeHelper next : TIMRADEdges) {
			if (next.isPassable() && next.getNeighbours().first().equals(neighbourId)) {
				result.add(next);
			}
		}

		return result;
	}

	public Set<TIMRADEdgeHelper> getPassableEdge() {
		Set<TIMRADEdgeHelper> result = new HashSet<>();

		for (TIMRADEdgeHelper next : TIMRADEdges) {
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

			return getPassableEdge().size() > 1;
		} else {
			List<TIMRADBlockadeHelper> blockades = new LinkedList<>(getTIMRADBlockades());

			for (TIMRADEscapePointHelper next : getEscapePoint(this, 500)) {
				blockades.removeAll(next.getRelateBlockade());
			}

			if (blockades.isEmpty())
				return true;
			return false;
		}
	}

	public boolean isEntrancePassable() {
		return false;
	}

	public boolean isAllEdgePassable() {
		for (TIMRADEdgeHelper next : TIMRADEdges) {
			if (!next.isPassable())
				return false;
		}
		return true;
	}

	public boolean isOneEdgeUnpassable() {
		int count = 0;
		for (TIMRADEdgeHelper next : TIMRADEdges) {
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

	public boolean isEntranceNeighbour(EntranceHelper entrance) {
		for (EntityID next : selfRoad.getNeighbours()) {
			StandardEntity neig = world.getEntity(next);
			if (neig instanceof Road && entrance.containsKey((Road) neig))
				return true;
		}
		return false;
	}

	public void setEntrance(boolean entrance) {
		this.isEntrance = entrance;
	}

	public Pair<Line2D, Line2D> getPfClearLine(TIMRADRoadHelper road) {

		if (this.pfClearLines != null)
			return this.pfClearLines;

		if (road.getTIMRADEdges().size() != 4)
			return null;
		if (road.isAllEdgePassable())
			return null;

		TIMRADEdgeHelper edge_1 = road.getTIMRADEdges().get(0);
		TIMRADEdgeHelper edge_2 = road.getTIMRADEdges().get(1);
		TIMRADEdgeHelper edge_3 = road.getTIMRADEdges().get(2);
		TIMRADEdgeHelper edge_4 = road.getTIMRADEdges().get(3);

		Line2D line_1 = null, line_2 = null, line_3 = null, line_4 = null;

		if (edge_1.isPassable() && edge_3.isPassable()) {
			roadCenterLine = new Line2D(edge_1.getMiddlePoint(), edge_3.getMiddlePoint());

			Point2D perpendicular_1, perpendicular_2;

			Pair<Double, Boolean> dis = ptSegDistSq(edge_2.getLine(), edge_1.getStart());
			if (dis.second().booleanValue()) { // the point is out the range of
												// this line
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

	public Area getPfClearArea(TIMRADRoadHelper road) {

		if (this.pfClearArea != null)
			return pfClearArea;

		if (road.getTIMRADEdges().size() != 4)
			return null;
		if (road.isAllEdgePassable())
			return null;

		TIMRADEdgeHelper edge_1 = road.getTIMRADEdges().get(0);
		TIMRADEdgeHelper edge_2 = road.getTIMRADEdges().get(1);
		TIMRADEdgeHelper edge_3 = road.getTIMRADEdges().get(2);
		TIMRADEdgeHelper edge_4 = road.getTIMRADEdges().get(3);

		Polygon area = new Polygon();

		Line2D line_1 = null, line_2 = null;

		if (edge_1.isPassable() && edge_3.isPassable()) {
			roadCenterLine = new Line2D(edge_1.getMiddlePoint(), edge_3.getMiddlePoint());
			Point2D perpendicular_1, perpendicular_2;

			Pair<Double, Boolean> dis = ptSegDistSq(edge_2.getLine(), edge_1.getStart());
			if (!dis.second().booleanValue()) { // the point is out the range of
												// this line
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
		area.addPoint((int) end_1.getX(), (int) end_1.getY());
		area.addPoint((int) end_2.getX(), (int) end_2.getY());

		end_1 = (new Line2D(mid_1, line_1.getEndPoint())).getPoint(rate_1);
		end_2 = (new Line2D(mid_2, line_2.getEndPoint())).getPoint(rate_2);

		// the order of the following two lines should not be change
		area.addPoint((int) end_2.getX(), (int) end_2.getY());
		area.addPoint((int) end_1.getX(), (int) end_1.getY());

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

	public List<TIMRADEscapePointHelper> getEscapePoint(TIMRADRoadHelper road, int threshold) {
		List<TIMRADEscapePointHelper> m_p_points = new LinkedList<>();

		for (TIMRADBlockadeHelper next : road.getTIMRADBlockades()) {
			if (next == null)
				continue;
			Polygon expan = next.getPolygon();

			for (TIMRADEdgeHelper TIMRADEdge : road.getTIMRADEdges()) {
				TIMRADEscapePointHelper p = findPoints(TIMRADEdge, expan, next);
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

	private TIMRADEscapePointHelper findPoints(TIMRADEdgeHelper TIMRADEdge, Polygon expan, TIMRADBlockadeHelper next) {
		if (TIMRADEdge.isPassable()) {
		} else {
			if (hasIntersection(expan, TIMRADEdge.getLine())) {
				return null;
			}
			double minDistance = Double.MAX_VALUE, distance;
			Pair<Integer, Integer> minDistanceVertex = null;

			for (Pair<Integer, Integer> vertex : next.getVertexesList()) {

				Pair<Double, Boolean> dis = ptSegDistSq(TIMRADEdge.getStart().getX(), TIMRADEdge.getStart().getY(),
				TIMRADEdge.getEnd().getX(), TIMRADEdge.getEnd().getY(), vertex.first(), vertex.second());

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

			return new TIMRADEscapePointHelper(middlePoint, lin, next);
		}

		return null;
	}

	private void filter(TIMRADRoadHelper road, List<TIMRADEscapePointHelper> m_p_points, int threshold) {
		Mark: for (Iterator<TIMRADEscapePointHelper> itor = m_p_points.iterator(); itor.hasNext();) {

			TIMRADEscapePointHelper m_p = itor.next();
			for (TIMRADEdgeHelper edge : road.getTIMRADEdges()) {
				if (edge.isPassable())
					continue;
				if (contains(edge.getLine(), m_p.getUnderlyingPoint(), threshold / 2)) {
					itor.remove();
					continue Mark;
				}
			}

			for (TIMRADBlockadeHelper blockade : road.getTIMRADBlockades()) {
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
					} else if (shouldRemove) {
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

		double pos = java.awt.geom.Line2D.ptSegDist(line.getOrigin().getX(), line.getOrigin().getY(),
				line.getEndPoint().getX(), line.getEndPoint().getY(), point.getX(), point.getY());
		if (pos <= threshold)
			return true;

		return false;
	}

	private Pair<Double, Boolean> ptSegDistSq(Line2D line, Point2D point) {
		return ptSegDistSq((int) line.getOrigin().getX(), (int) line.getOrigin().getY(),
				(int) line.getEndPoint().getX(), (int) line.getEndPoint().getY(), (int) point.getX(),
				(int) point.getY());
	}

	private Pair<Double, Boolean> ptSegDistSq(double x1, double y1, double x2, double y2, double px, double py) {

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
			return new Pair<Double, Boolean>(Math.sqrt(lenSq), true);
		} else {
			return new Pair<Double, Boolean>(Math.sqrt(lenSq), false);
		}
	}


	public boolean hasIntersection(Polygon polygon, Line2D line) {
		List<Line2D> polyLines = getLines(polygon);
		for (Line2D ln : polyLines) {

			Line2D line_1 = new Line2D(line.getOrigin().getX(),
					line.getOrigin().getY(), line.getEndPoint().getX(), line.getEndPoint().getY());

			Line2D line_2 = new Line2D(ln.getOrigin().getX(), ln.getOrigin().getY(),
					ln.getOrigin().getX(), ln.getOrigin().getY());

			if (Line2D.intersects(line_1, line_2)) {

				return true;
			}
		}
		return false;
	}

	private List<Line2D> getLines(Polygon polygon) {
		List<Line2D> lines = new LinkedList<>();
		int count = polygon.npoints;
		for (int i = 0; i < count; i++) {
			int j = (i + 1) % count;
			Point2D p1 = new Point2D(polygon.xpoints[i],
					polygon.ypoints[i]);
			Point2D p2 = new Point2D(polygon.xpoints[j],
					polygon.ypoints[j]);
			Line2D line = new Line2D(p1, p2);
			lines.add(line);
		}
		return lines;
	}

	private Point getMiddle(Pair<Integer, Integer> first, Point2D second) {
		int x = first.first() + (int) second.getX();
		int y = first.second() + (int) second.getY();

		return new Point(x / 2, y / 2);
	}

	private Point getMiddle(Point2D first, Point2D second) {
		int x = (int) (first.getX() + second.getX());
		int y = (int) (first.getY() + second.getY());

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

	public void setTIMRADBlockades(List<TIMRADBlockadeHelper> blockades) {
		this.TIMRADBlockades.clear();
		this.TIMRADBlockades.addAll(blockades);
	}

	public List<TIMRADEdgeHelper> getTIMRADEdges() {
		return this.TIMRADEdges;
	}

	public List<TIMRADBlockadeHelper> getTIMRADBlockades() {
		return this.TIMRADBlockades;
	}

	public <T extends StandardEntity> T getEntity(WorldInfo world, EntityID id, Class<T> c) {
		StandardEntity entity;
		entity = world.getEntity(id);
		if (c.isInstance(entity)) {
			T castedEntity = c.cast(entity);

			return castedEntity;
		} else {
			return null;
		}
	}
}
