package TIMRAD_2025.standard;

import adf.core.agent.info.WorldInfo;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.io.Serializable;
import java.util.*;

public class TIMRADLineOfSightPerception {
	private static final int VIEW_DISTANCE = 30000;
	private static final int RAY_COUNT = 72;
	private static final IntersectionSorter INTERSECTION_SORTER = new IntersectionSorter();

	private WorldInfo world;

	public TIMRADLineOfSightPerception(WorldInfo model) {
		this.world = model;
	}

	@Override
	public String toString() {
		return "Line of sight perception";
	}

	public List<EntityID> getVisibleAreas(EntityID areaID) {
		Area area = (Area) world.getEntity(areaID);
		List<EntityID> result = new LinkedList<EntityID>();
		Pair<Integer, Integer> location = world.getLocation(areaID);
		if (location != null) {
			int x = location.first(), y = location.second();
			Point2D point = new Point2D(x, y);
			Collection<StandardEntity> nearby = world.getObjectsInRange(x, y, VIEW_DISTANCE);
			Collection<StandardEntity> visible = findVisibleAreas(area, point, nearby);
			for (StandardEntity next : visible) {
				if (next instanceof Area) {
					result.add(next.getID());
				}
			}
		}
		return result;
	}

	private Collection<StandardEntity> findVisibleAreas(Area area, Point2D location,
			Collection<StandardEntity> nearby) {
		Collection<LineInfo> lines = getAllLines(nearby);
		double dAngle = Math.PI * 2 / RAY_COUNT;
		Collection<StandardEntity> result = new HashSet<StandardEntity>();
		for (int i = 0; i < RAY_COUNT; ++i) {
			double angle = i * dAngle;
			Vector2D vector = new Vector2D(Math.sin(angle), Math.cos(angle)).scale(VIEW_DISTANCE);
			TIMRADRay ray = new TIMRADRay(new Line2D(location, vector), lines);
			for (LineInfo hit : ray.getLinesHit()) {
				StandardEntity e = hit.getEntity();
				result.add(e);
			}
		}
		return result;
	}

	private Collection<LineInfo> getAllLines(Collection<StandardEntity> entities) {
		Collection<LineInfo> result = new HashSet<LineInfo>();
		for (StandardEntity next : entities) {
			if (next instanceof Building) {
				for (Edge edge : ((Building) next).getEdges()) {
					Line2D line = edge.getLine();
					result.add(new LineInfo(line, next, !edge.isPassable()));
				}
			}
			if (next instanceof Road) {
				for (Edge edge : ((Road) next).getEdges()) {
					Line2D line = edge.getLine();
					result.add(new LineInfo(line, next, false));
				}
			} else if (next instanceof Blockade) {
				int[] apexes = ((Blockade) next).getApexes();
				List<Point2D> points = GeometryTools2D.vertexArrayToPoints(apexes);
				List<Line2D> lines = GeometryTools2D.pointsToLines(points, true);
				for (Line2D line : lines) {
					result.add(new LineInfo(line, next, false));
				}
			} else {
				continue;
			}
		}
		return result;
	}

	private static class TIMRADRay {
		private Line2D ray;
		private double length;
		private List<LineInfo> hit;

		public TIMRADRay(Line2D ray, Collection<LineInfo> otherLines) {
			this.ray = ray;
			List<Pair<LineInfo, Double>> intersections = new LinkedList<Pair<LineInfo, Double>>();
			// Find intersections with other lines
			for (LineInfo other : otherLines) {
				double d1 = ray.getIntersection(other.getLine());
				double d2 = other.getLine().getIntersection(ray);
				if (d2 >= 0 && d2 <= 1 && d1 > 0 && d1 <= 1) {
					intersections.add(new Pair<LineInfo, Double>(other, d1));
				}
			}
			Collections.sort(intersections, INTERSECTION_SORTER);
			hit = new LinkedList<LineInfo>();
			length = 1;
			for (Pair<LineInfo, Double> next : intersections) {
				LineInfo l = next.first();
				hit.add(l);
				if (l.isBlocking()) {
					length = next.second();
					break;
				}
			}
		}

		@SuppressWarnings("unused")
		public Line2D getRay() {
			return ray;
		}

		@SuppressWarnings("unused")
		public double getVisibleLength() {
			return length;
		}

		public List<LineInfo> getLinesHit() {
			return Collections.unmodifiableList(hit);
		}
	}

	private static class LineInfo {
		private Line2D line;
		private StandardEntity entity;
		private boolean blocking;

		public LineInfo(Line2D line, StandardEntity entity, boolean blocking) {
			this.line = line;
			this.entity = entity;
			this.blocking = blocking;
		}

		public Line2D getLine() {
			return line;
		}

		public StandardEntity getEntity() {
			return entity;
		}

		public boolean isBlocking() {
			return blocking;
		}
	}

	@SuppressWarnings("serial")
	private static class IntersectionSorter implements Comparator<Pair<LineInfo, Double>>, Serializable {
		@Override
		public int compare(Pair<LineInfo, Double> a, Pair<LineInfo, Double> b) {
			double d1 = a.second();
			double d2 = b.second();
			if (d1 < d2) {
				return -1;
			}
			if (d1 > d2) {
				return 1;
			}
			return 0;
		}
	}
}
