package TIMRAD_2025.extaction;

import TIMRAD_2025.helptool.TIMRADConstants;
import TIMRAD_2025.debugger.DebugHelper;
import TIMRAD_2025.module.algorithm.CompositeConvexHull;
import TIMRAD_2025.module.complex.fb.tools.FbUtilities;
import TIMRAD_2025.module.complex.pf.GuidelineCreator;
import TIMRAD_2025.helptool.Ruler;
import TIMRAD_2025.helptool.Util;
import TIMRAD_2025.helptool.TIMRADWorldHelper;
import TIMRAD_2025.helptool.object.TIMRADEdge;
import TIMRAD_2025.helptool.object.TIMRADLineOfSightPerception;
import TIMRAD_2025.helptool.object.TIMRADRoad;
import adf.core.agent.action.Action;
import adf.core.agent.action.common.ActionMove;
import adf.core.agent.info.WorldInfo;
import com.mrl.debugger.remote.dto.StuckDto;
import rescuecore2.misc.Pair;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.misc.geometry.Vector2D;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;


import java.awt.*;
import java.util.List;
import java.util.*;
import java.util.stream.Collectors;

/**
 * @author: Guanyu-Cai
 * @Date: 03/10/2020
 */
public class StuckHelper {
    private GuidelineCreator guidelineCreator;
    private TIMRADWorldHelper world;
    private TIMRADLineOfSightPerception lineOfSightPerception;
    private WorldInfo worldInfo;
    public StuckHelper(TIMRADWorldHelper world, WorldInfo worldInfo) {
        this.world = world;
        this.worldInfo = worldInfo;
        this.lineOfSightPerception = new TIMRADLineOfSightPerception(world);
    }
    private Point2D getMidPoint(Edge edge) {
        if(edge != null) {
            double midX = (edge.getStartX() + edge.getEndX()) / 2;
            double midY = (edge.getStartY() + edge.getEndY()) / 2;
            Point2D point = new Point2D(midX, midY);
            return point;
        }
        return null;
    }
    private Edge getOppositeEdge(Road road,Edge original) {
        List<Edge> edges = new ArrayList<>();
        for(Edge edge : road.getEdges()){
            if(edge.isPassable() && !this.getMidPoint(edge).equals(this.getMidPoint(original))){
                edges.add(edge);
            }
        }
        if(edges.size() > 0) {
            Point2D roadCenter = new Point2D(road.getX(), road.getY());
            Point2D originalMid = this.getMidPoint(original);
            Vector2D standardDirection = new Vector2D(roadCenter.getX() - originalMid.getX(), roadCenter.getY() - originalMid.getY());
            Edge answerEdge = null;
            double minAngle = Double.MAX_VALUE;
            for (int i = 0; i < edges.size(); ++i) {
                Point2D mid = this.getMidPoint(edges.get(i));
                Vector2D testDirection = new Vector2D(mid.getX() - originalMid.getX(), mid.getY() - originalMid.getY());
                double angle = GeometryTools2D.getAngleBetweenVectors(standardDirection, testDirection);
                if (angle < minAngle) {
                    minAngle = angle;
                    answerEdge = edges.get(i);
                }
            }
            return answerEdge;
        }
        return null;
    }
    private double getDistance(double fromX, double fromY, double toX, double toY) {
        double dx = fromX - toX;
        double dy = fromY - toY;
        return Math.hypot(dx, dy);
    }

    private Line2D getProperLine(Road road) {
        List<Edge> edges = new ArrayList<>();
        List<Edge> reservedEdges = new ArrayList<>();
        for (Edge edge : road.getEdges()) {
            Boolean edgeToEntrance = false;
            for(EntityID neighbour : road.getNeighbours()){
                Edge neighbourEdge = road.getEdgeTo(neighbour);
                if(!(this.worldInfo.getEntity(neighbour) instanceof Road)&& this.getMidPoint(edge).equals(this.getMidPoint(neighbourEdge))){
                    edgeToEntrance = true;
                    break;
                }
            }
            if(edge.isPassable()){
                reservedEdges.add(edge);
                if(!edgeToEntrance){
                    edges.add(edge);
                }
            }
        }

        Point2D start = null;
        Point2D end = null;
        double max = Double.MIN_VALUE;
        if(edges != null && edges.size() > 1) {
            for (Edge edge : edges) {
                Edge opposite = this.getOppositeEdge(road, edge);
                if(opposite != null) {
                    Point2D p1 = this.getMidPoint(edge);
                    Point2D p2 = this.getMidPoint(opposite);
                    double dist = this.getDistance(p1.getX(), p1.getY(), p2.getX(), p2.getY());
                    if (dist > max) {
                        max = dist;
                        start = p1;
                        end = p2;
                    }
                }
            }
            if (start != null && end != null) {
                Line2D guideline = new Line2D(start, end);
                return guideline;
            }
        }else if(edges != null && edges.size() == 1){
            Edge only = edges.get(0);
            Point2D onlyMid = this.getMidPoint(only);
            Point2D roadCenter = new Point2D(road.getX(),road.getY());
            List<Edge> allOtherEdges = new ArrayList<>();
            for(Edge e : road.getEdges()){
                if(!this.getMidPoint(e).equals(this.getMidPoint(only))){
                    allOtherEdges.add(e);
                }
            }
            Vector2D standardDirection = new Vector2D(roadCenter.getX() - onlyMid.getX(), roadCenter.getY() - onlyMid.getY());
            Point2D answerPoint = null;
            double minAngle = Double.MAX_VALUE;
            for (int i = 0; i < allOtherEdges.size(); ++i) {
                Point2D mid = this.getMidPoint(allOtherEdges.get(i));
                Vector2D testDirection = new Vector2D(mid.getX() - onlyMid.getX(), mid.getY() - onlyMid.getY());
                double angle = GeometryTools2D.getAngleBetweenVectors(standardDirection, testDirection);
                if (angle < minAngle) {
                    minAngle = angle;
                    answerPoint = mid;
                }
            }
            Line2D guideline = new Line2D(onlyMid,answerPoint);
            return guideline;
        }
        //in case of all the neighbours are entrance
        else {
            if(reservedEdges != null && reservedEdges.size() > 1) {
                for (Edge edge : reservedEdges) {
                    Edge opposite = this.getOppositeEdge(road, edge);
                    Point2D p1 = this.getMidPoint(edge);
                    Point2D p2 = this.getMidPoint(opposite);
                    double dist = this.getDistance(p1.getX(), p1.getY(), p2.getX(), p2.getY());
                    if (dist > max) {
                        max = dist;
                        start = p1;
                        end = p2;
                    }
                }
                if (start != null && end != null) {
                    Line2D guideline = new Line2D(start, end);
                    return guideline;
                }
            }else if(reservedEdges != null && reservedEdges.size() == 1){
                Edge only = reservedEdges.get(0);
                Point2D onlyMid = this.getMidPoint(only);
                Point2D roadCenter = new Point2D(road.getX(),road.getY());
                List<Edge> allOtherEdges = new ArrayList<>();
                for(Edge e : road.getEdges()){
                    if(!this.getMidPoint(e).equals(this.getMidPoint(only))){
                        allOtherEdges.add(e);
                    }
                }
                allOtherEdges.remove(only);
                Vector2D standardDirection = new Vector2D(roadCenter.getX() - onlyMid.getX(), roadCenter.getY() - onlyMid.getY());
                Point2D answerPoint = null;
                double minAngle = Double.MAX_VALUE;
                for (int i = 0; i < allOtherEdges.size(); ++i) {
                    Point2D mid = this.getMidPoint(allOtherEdges.get(i));
                    Vector2D testDirection = new Vector2D(mid.getX() - onlyMid.getX(), mid.getY() - onlyMid.getY());
                    double angle = GeometryTools2D.getAngleBetweenVectors(standardDirection, testDirection);
                    if (angle < minAngle) {
                        minAngle = angle;
                        answerPoint = mid;
                    }
                }
                Line2D guideline = new Line2D(onlyMid,roadCenter);
                return guideline;
            }
        }
        return null;
    }
    public Action calc(List<EntityID> path) {
        if (path == null || path.size() < 2) {
            return null;
        }

        List<EntityID> selfAndNeighborBlockades = getSelfAndNeighborBlockades();
        EntityID nearestBlockade = getNearestBlockade();
        List<StandardEntity> nearBlockades = getBlockadesInRange(nearestBlockade, selfAndNeighborBlockades, TIMRADConstants.AGENT_PASSING_THRESHOLD);
        if (nearBlockades.isEmpty()) {
            return null;
        }
        CompositeConvexHull blockadesConvexHull = getConvexByBlockades(nearBlockades);

        StandardEntity selfPosition = world.getSelfPosition();
        Pair<Integer, Integer> selfLocation = world.getSelfLocation();
        Point2D locationPoint = new Point2D(selfLocation.first(), selfLocation.second());
        Point2D openPartCenter = null;
        Point2D edgeStart = null;
        Point2D edgeEnd = null;
        TIMRADEdge targetEdge = null;
        if (selfPosition instanceof Road) {
            TIMRADRoad TIMRADRoad = world.getTIMRADRoad(selfPosition);
            for (TIMRADEdge TIMRADEdge : TIMRADRoad.getTIMRADEdgesTo(path.get(1))) {
                if (!TIMRADEdge.isBlocked()) {
                    openPartCenter = TIMRADEdge.getOpenPartCenter();
                    edgeStart = TIMRADEdge.getStart();
                    edgeEnd = TIMRADEdge.getEnd();
                    targetEdge = TIMRADEdge;
                    break;
                }
            }
        }
        if(selfPosition instanceof Building){
            Building sd = (Building) selfPosition;
            for(EntityID ne : sd.getNeighbours()){
                StandardEntity neigbor = world.getEntity(ne);
                if(neigbor instanceof Road){
                    TIMRADRoad TIMRADRoad = world.getTIMRADRoad(neigbor);
                    for (TIMRADEdge TIMRADEdge : TIMRADRoad.getTIMRADEdgesTo(sd.getID())) {
                        if (!TIMRADEdge.isBlocked()) {
                            openPartCenter = TIMRADEdge.getOpenPartCenter();
                            edgeStart = TIMRADEdge.getStart();
                            edgeEnd = TIMRADEdge.getEnd();
                            targetEdge = TIMRADEdge;
                            break;
                        }
                    }
                }
            }
        }
        if (openPartCenter == null) {
            return null;
        }
        double x_dis=0,y_dist=0;
        if(edgeStart!=null)
            x_dis = (edgeEnd.getX()-edgeStart.getX())/8;
            y_dist = (edgeEnd.getY()-edgeStart.getY())/8;
        Line2D guideLine = new Line2D(locationPoint, openPartCenter);
        Point2D target = null;
        Set<Set<TIMRADLineOfSightPerception.TIMRADRay>> raysNotHits = new HashSet<>();
        Set<TIMRADLineOfSightPerception.TIMRADRay> raysNotHit1 = new HashSet<>();
        Collection<TIMRADRoad> targetValidRoads = new HashSet<>();
        if (!Util.hasIntersectLine(blockadesConvexHull.getConvexPolygon(), guideLine)) {
            target = Util.clipLine(guideLine, world.getConfig().maxRayDistance).getEndPoint();
            if (target.getX() > world.getMaxX() || target.getX() < 0 || target.getY() > world.getMaxY() || target.getY() < 0) {
                target = Util.improveLine(guideLine, TIMRADConstants.AGENT_SIZE).getEndPoint();
            }
        } else {
            TIMRADRoad TIMRADRoad,oppositePassableEdgeRoad = null;
            if(selfPosition instanceof Road) {
                TIMRADRoad = world.getTIMRADRoad(selfPosition);
                targetValidRoads.add(TIMRADRoad);
                oppositePassableEdgeRoad = TIMRADRoad.getOppositePassableEdgeRoad(targetEdge);
            }
            if(selfPosition instanceof Building){
                Building sd = (Building) selfPosition;
                for(EntityID ne : sd.getNeighbours()){
                    StandardEntity neigbor = world.getEntity(ne);
                    if(neigbor instanceof Road){
                        TIMRADRoad = world.getTIMRADRoad(neigbor);
                        for (TIMRADEdge TIMRADEdge : TIMRADRoad.getTIMRADEdgesTo(sd.getID())) {
                            if (!TIMRADEdge.isBlocked()) {
                                targetValidRoads.add(TIMRADRoad);
                                break;
                            }
                        }
                    }
                }
            }
            if (oppositePassableEdgeRoad != null) {
                targetValidRoads.add(oppositePassableEdgeRoad);
            }
            int distance = 30000000;
            Line2D guidline = null;
            double x_dis_2 = 0,y_dis_2 = 0;
            raysNotHit1 = lineOfSightPerception.findRaysNotHit(locationPoint, nearBlockades, distance);
            Set<TIMRADLineOfSightPerception.TIMRADRay> raysNotHit2 = lineOfSightPerception.findRaysNotHit(openPartCenter, nearBlockades, distance);
            for(int i = 0;i <= 8;i++) {
                Point2D now_point = new Point2D(edgeStart.getX()+x_dis*i,edgeStart.getY()+y_dist*i);
                Set<TIMRADLineOfSightPerception.TIMRADRay> raysNotHit3 = lineOfSightPerception.findRaysNotHit(now_point, nearBlockades, distance);
                if(guidline != null)
                {
                    Point2D now_line = new Point2D(guidline.getOrigin().getX()+x_dis_2*i,guidline.getOrigin().getY()+y_dis_2*i);
                    Set<TIMRADLineOfSightPerception.TIMRADRay> raysNotHit4 = lineOfSightPerception.findRaysNotHit(now_line, nearBlockades, distance);
                    raysNotHits.add(raysNotHit4);
                }
                raysNotHits.add(raysNotHit3);
            }
            raysNotHits.add(raysNotHit2);
            for (Set<TIMRADLineOfSightPerception.TIMRADRay> next : raysNotHits) {
                List<Point2D> validIntersections = getValidIntersections(raysNotHit1, next, targetValidRoads);
                if (!validIntersections.isEmpty()) {
                    validIntersections.sort(new DistanceComparator(locationPoint));
                    target = Util.improveLine(new Line2D(locationPoint, validIntersections.get(0)), TIMRADConstants.AGENT_PASSING_THRESHOLD).getEndPoint();
                }
            }
        }
        if (DebugHelper.DEBUG_MODE) {
            StuckDto stuckDto = new StuckDto();
            if (target != null) {
                stuckDto.setAgentId(world.getSelfHuman().getID().getValue());
                stuckDto.setBlockadesConvexHull(blockadesConvexHull.getConvexPolygon());
                stuckDto.setTarget(Util.convertPoint(target));
                stuckDto.setGuideLine(Util.convertLine2(guideLine));
                HashSet<java.awt.geom.Line2D> line2DS = new HashSet<>();
                for (Set<TIMRADLineOfSightPerception.TIMRADRay> e : raysNotHits) {
                    line2DS.addAll(e.stream().map(a -> new java.awt.geom.Line2D.Double(a.getRay().getOrigin().getX(), a.getRay().getOrigin().getY(),
                            a.getRay().getEndPoint().getX(), a.getRay().getEndPoint().getY())).collect(Collectors.toSet()));
                }
                HashSet<java.awt.geom.Line2D> selfLine2DS = new HashSet<>();
                selfLine2DS.addAll(raysNotHit1.stream().map(a -> new java.awt.geom.Line2D.Double(a.getRay().getOrigin().getX(), a.getRay().getOrigin().getY(),
                        a.getRay().getEndPoint().getX(), a.getRay().getEndPoint().getY())).collect(Collectors.toSet()));
                stuckDto.setRaysNotHits(line2DS);
                stuckDto.setSelfRaysNotHits(selfLine2DS);
                stuckDto.setOpenPartCenter(Util.convertPoint(openPartCenter));
                stuckDto.setTargetValidRoads(Util.fetchIdValueFromElementIds(targetValidRoads.stream().map(TIMRADRoad::getId).collect(Collectors.toSet())));
            }
            DebugHelper.VD_CLIENT.drawAsync(world.getSelfHuman().getID().getValue(), "StuckDtoLayer", stuckDto);
        }
        return moveToPoint(target);
    }

    private List<Point2D> getValidIntersections(Set<TIMRADLineOfSightPerception.TIMRADRay> a, Set<TIMRADLineOfSightPerception.TIMRADRay> b,
                                                Collection<TIMRADRoad> targetValidRoads) {
        List<Point2D> intersections = Util.getSegmentIntersections(a, b);
        if (!intersections.isEmpty()) {
            filterIntersectionsNotInRoads(intersections, targetValidRoads);
        }
        return intersections;
    }

    private Collection<TIMRADRoad> getSelfAndNearestRoad() {
        TIMRADRoad TIMRADRoad = world.getTIMRADRoad(world.getSelfPosition());
        Collection<TIMRADRoad> result = new HashSet<>();
        result.add(TIMRADRoad);
        result.add(world.getNearestNeighborRoad());
        return result;
    }

    private void filterIntersectionsNotInRoads(Collection<Point2D> points , Collection<TIMRADRoad> roads) {
        Collection<Point2D> toRemove = new HashSet<>();
        for (Point2D point : points) {
            boolean contain = false;
            for (TIMRADRoad road : roads) {
                Polygon polygon = road.getPolygon();
                if (polygon.contains(point.getX(), point.getY())) {
                    contain = true;
                    break;
                }
            }
            if (!contain) {
                toRemove.add(point);
            }
        }
        points.removeAll(toRemove);
    }

    private Action moveToPoint(Point2D target) {
        if (target == null) {
            return null;
        }
        List<EntityID> path = new ArrayList<>();
        path.add(world.getSelfPosition().getID());
        return new ActionMove(path, (int) target.getX(), (int) target.getY());
    }

    private CompositeConvexHull getConvexByBlockades(List<StandardEntity> blockades) {
        CompositeConvexHull convexHull = new CompositeConvexHull();
        for (StandardEntity entity : blockades) {
            Blockade blockade;
            if (entity instanceof Blockade) {
                blockade = (Blockade) entity;
                for (int i = 0; i < blockade.getApexes().length; i += 2) {
                    convexHull.addPoint(blockade.getApexes()[i], blockade.getApexes()[i + 1]);
                }
            }
        }
        return convexHull;
    }

    private List<EntityID> getSelfAndNeighborBlockades() {
        StandardEntity selfPosition = world.getSelfPosition();
        List<EntityID> blockadeIDs = new ArrayList<>();

        if (selfPosition instanceof Road) {
            Road road = (Road) selfPosition;
            for (StandardEntity next : world.getEntities(road.getNeighbours())) {
                if (next instanceof Road) {
                    Road neighbour = (Road) next;
                    if (neighbour.isBlockadesDefined()) {
                        blockadeIDs.addAll(neighbour.getBlockades());
                    }
                }
            }
            blockadeIDs.addAll(road.getBlockades());
            return blockadeIDs;
        }
        if(selfPosition instanceof  Building){
            Building building = (Building) selfPosition;
            for (StandardEntity next : world.getEntities(building.getNeighbours())) {
                if (next instanceof Road) {
                    Road neighbour = (Road) next;
                    if (neighbour.isBlockadesDefined()) {
                        blockadeIDs.addAll(neighbour.getBlockades());
                    }
                }
            }
            return blockadeIDs;
        }
        return null;
    }

    private EntityID getNearestBlockade() {
        List<EntityID> selfAndNeighborBlockades = getSelfAndNeighborBlockades();
        EntityID nearest = null;
        if (selfAndNeighborBlockades != null) {
            nearest = FbUtilities.getNearest(world, selfAndNeighborBlockades, world.getSelfHuman().getID());
        }
        return nearest;
    }

    private List<StandardEntity> getBlockadesInRange(EntityID entity, List<EntityID> entities, int range) {
        List<StandardEntity> inRangeBlockades = new ArrayList<>();
        if (entities == null || entities.isEmpty()) {
            return inRangeBlockades;
        }
        Blockade anchorBlockade = (Blockade) world.getEntity(entity);
        for (EntityID blockadeID : entities) {
            StandardEntity standardEntity = world.getEntity(blockadeID);
            if (!(standardEntity instanceof Blockade)) {
                continue;
            }
            Blockade blockade = (Blockade) standardEntity;
            double dist = Ruler.getDistance(Util.getPolygon(blockade.getApexes()), Util.getPolygon(anchorBlockade.getApexes()));
            if (dist < range) {
                inRangeBlockades.add(blockade);
            }
        }
        return inRangeBlockades;
    }

    private static class DistanceComparator implements Comparator<Point2D> {
        private Point2D reference;
        public DistanceComparator(Point2D reference) {
            this.reference = reference;
        }
        @Override
        public int compare(Point2D a, Point2D b) {
            int d1 = (int) Ruler.getDistance(reference, a);
            int d2 = (int) Ruler.getDistance(reference, b);
            return d1 - d2;
        }
    }
}
