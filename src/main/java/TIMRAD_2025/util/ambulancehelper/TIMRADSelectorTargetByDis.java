package TIMRAD_2025.util.ambulancehelper;

//import CSU_Yunlu_2021.util.DistanceSorter;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.awt.geom.Point2D;
import java.util.List;
import java.util.*;

public class TIMRADSelectorTargetByDis {

    private Map<EntityID, TIMRADTarget> targetsMap;
    private double threshold = Double.MAX_VALUE;// A threshold for selecting victim
    private int rescueRange;
    protected boolean isMapHuge = false;
    protected boolean isMapMedium = false;
    protected boolean isMapSmall = false;
    public static final double MEAN_VELOCITY_OF_MOVING = 31445.392;
    private static final double AGENT_CAN_MOVE = 7000.0;
    protected int minX, minY, maxX, maxY;
    private TIMRADTarget previousTarget = null;
    private int clusterIndex;
    private Map<Integer, Pair<Integer, Integer>> centersMap;
    Map<Integer, Pair<Integer, Integer>> clusterCenterMap;

    private WorldInfo worldInfo;
    private ScenarioInfo scenarioInfo;
    private AgentInfo agentInfo;
    private Clustering clustering;
    private PathPlanning pathPlanning;

    public TIMRADSelectorTargetByDis(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {

        this.worldInfo = wi;
        this.scenarioInfo = si;
        this.agentInfo = ai;

        switch (scenarioInfo.getMode()) {
            case PRECOMPUTATION_PHASE:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning", "adf.core.sample.module.algorithm.SamplePathPlanning");
                this.clustering = moduleManager.getModule("SampleHumanDetector.Clustering", "adf.core.sample.module.algorithm.SampleKMeans");
                break;
            case PRECOMPUTED:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning", "adf.core.sample.module.algorithm.SamplePathPlanning");
                this.clustering = moduleManager.getModule("SampleHumanDetector.Clustering", "adf.core.sample.module.algorithm.SampleKMeans");
                break;
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning", "adf.core.sample.module.algorithm.SamplePathPlanning");
                this.clustering = moduleManager.getModule("SampleHumanDetector.Clustering", "adf.core.sample.module.algorithm.SampleKMeans");
                break;
        }


        targetsMap = new HashMap<>();
        clusterCenterMap = new HashMap<>();
        calculateMapDimensions();
        verifyMap();
        initializeRescueRange();
    }


    private void calculateMapDimensions() {
        this.minX = Integer.MAX_VALUE;
        this.maxX = Integer.MIN_VALUE;
        this.minY = Integer.MAX_VALUE;
        this.maxY = Integer.MIN_VALUE;
        Pair<Integer, Integer> pos;
        List<StandardEntity> invalidEntities = new ArrayList<>();
        for (StandardEntity standardEntity : worldInfo.getAllEntities()) {
            pos = standardEntity.getLocation(worldInfo.getRawWorld());
            if (pos.first() == Integer.MIN_VALUE || pos.first() == Integer.MAX_VALUE || pos.second() == Integer.MIN_VALUE || pos.second() == Integer.MAX_VALUE) {
                invalidEntities.add(standardEntity);
                continue;
            }
            if (pos.first() < this.minX)
                this.minX = pos.first();
            if (pos.second() < this.minY)
                this.minY = pos.second();
            if (pos.first() > this.maxX)
                this.maxX = pos.first();
            if (pos.second() > this.maxY)
                this.maxY = pos.second();
        }
        if (!invalidEntities.isEmpty()) {
            System.out.println("##### WARNING: There is some invalid entities ====> " + invalidEntities.size());
        }
    }

    private int getMapWidth() {
        return maxX - minX;
    }

    private int getMapHeight() {
        return maxY - minY;
    }


    private void verifyMap() {

        double mapDimension = Math.hypot(getMapWidth(), getMapHeight());

        double rate = mapDimension / MEAN_VELOCITY_OF_MOVING;

        if (rate > 60) {
            isMapHuge = true;
        } else if (rate > 30) {
            isMapMedium = true;
        } else {
            isMapSmall = true;
        }


    }

    private void initializeRescueRange() {
        int perceptionLosMaxDistance = scenarioInfo.getPerceptionLosMaxDistance();
        if (isMapSmall) {
            rescueRange = perceptionLosMaxDistance * 2;
        } else if (isMapMedium) {
            rescueRange = perceptionLosMaxDistance * 4;
        } else { //if map is huge
            rescueRange = perceptionLosMaxDistance * 6;
        }
    }


    /**
     * Finds best target between specified possible targetsMap
     *
     * @return best target to select
     */
    public EntityID nextTarget(Set<StandardEntity> victims) {

        this.clusterIndex = clustering.getClusterIndex(this.agentInfo.getID());

        Collection<StandardEntity> elements = clustering.getClusterEntities(this.clusterIndex);

        calculateMapCenters(this.clusterIndex, elements);

        targetsMap.clear();

        if (previousTarget != null && !victims.contains(worldInfo.getEntity(previousTarget.getVictimID()))) {
            previousTarget = null;
        }


        refreshTargetsMap(victims, targetsMap);

        calculateDecisionParameters(victims, targetsMap);

        calculateVictimsCostValue();


        TIMRADTarget bestTarget = null;
        bestTarget = findBestVictim(targetsMap, elements);

        //considering inertia for the current target to prevent loop in target selection
        if (previousTarget != null && victims.contains(worldInfo.getEntity(previousTarget.getVictimID()))) {
            if (bestTarget != null && !bestTarget.getVictimID().equals(previousTarget.getVictimID())) {
                Human bestHuman = (Human) worldInfo.getEntity(bestTarget.getVictimID());
                Human previousHuman = (Human) worldInfo.getEntity(previousTarget.getVictimID());

                double bestHumanCost = pathPlanning.setFrom(agentInfo.getPosition()).setDestination(bestHuman.getPosition()).calc().getDistance();
                double previousHumanCost = pathPlanning.setFrom(agentInfo.getPosition()).setDestination(previousHuman.getPosition()).calc().getDistance();
                if (previousHumanCost < bestHumanCost) {
                    bestTarget = previousTarget;
                }
            }
        }

        previousTarget = bestTarget;

        if (bestTarget != null) {
            return bestTarget.getVictimID();
        } else {
            return null;
        }

    }

    private void calculateMapCenters(int clusterIndex, Collection<StandardEntity> elements) {

        int minX = Integer.MAX_VALUE, minY = Integer.MAX_VALUE;
        int maxX = 0, maxY = 0;
        for (StandardEntity entity : elements) {
            Pair<Integer, Integer> location = worldInfo.getLocation(entity);
            minX = Math.min(location.first(), minX);
            minY = Math.min(location.second(), minY);

            maxX = Math.max(location.first(), maxX);
            maxY = Math.max(location.second(), maxY);
        }

        Pair<Integer, Integer> center = new Pair<>((minX + maxX) / 2, (minY + maxY) / 2);
        clusterCenterMap.put(clusterIndex, center);

    }

    private void refreshTargetsMap(Set<StandardEntity> victims, Map<EntityID, TIMRADTarget> targetsMap) {

        List<EntityID> toRemoves = new ArrayList<EntityID>();
        for (EntityID targetID : targetsMap.keySet()) {
            if (!victims.contains(worldInfo.getEntity(targetID))) {
                toRemoves.add(targetID);
            }
        }
        for (EntityID entityID : toRemoves) {
            targetsMap.remove(entityID);
        }
    }

    public TIMRADTarget findBestVictim(Map<EntityID, TIMRADTarget> targetsMap, Collection<StandardEntity> elements) {
        TIMRADTarget bestTarget = null;
        if (targetsMap != null && !targetsMap.isEmpty()) {

            double minValue = Double.MAX_VALUE;
            for (TIMRADTarget target : targetsMap.values()) {
                if (target.getDistanceToMe() <= rescueRange || elements.contains(worldInfo.getEntity(target.getPositionID()))) {
                    if (target.getDistanceToMe() < minValue) {
                        minValue = target.getDistanceToMe();
                        bestTarget = target;
                    }
                }
            }

        }

        return bestTarget;
    }

    private void calculateVictimsCostValue() {

        if (targetsMap != null && !targetsMap.isEmpty()) {
            double cost = 0;
            double rw = .9;//refuge Weight
            double pdw = 2.7;//my Partition Distance Weight
            double mdw = 1.5;//My Distance Weight
            double vsw = 1.5; //Victim Situation Weight

            for (TIMRADTarget target : targetsMap.values()) {
                cost = 1 + rw * target.getDistanceToRefuge() / MEAN_VELOCITY_OF_MOVING
                        + pdw * target.getDistanceToPartition() / MEAN_VELOCITY_OF_MOVING
                        + mdw * target.getDistanceToMe() / MEAN_VELOCITY_OF_MOVING
                        - vsw * target.getVictimSituation();
                target.setCost(cost);
            }

        }

    }

    /**
     * Calculates different parameters for a target such as distance of the target to nearest refuge, distance of target to
     * this agent partitions, distance of this agent to the target and situations of the target based on its BRD and DMG
     *
     * @param victims    victims to calculate their parameters
     * @param targetsMap map of previously found targets
     */
    private void calculateDecisionParameters(Set<StandardEntity> victims, Map<EntityID, TIMRADTarget> targetsMap) {

        TIMRADTarget target;
        Human human;
        if (victims != null && !victims.isEmpty()) {
            for (StandardEntity victim : victims) {
                target = targetsMap.get(victim.getID());
                human = (Human) worldInfo.getEntity(victim.getID());
                if (target == null) {
                    //creating a new CSU_Target object
                    target = new TIMRADTarget(victim.getID());

                    //set target position
                    target.setPositionID(human.getPosition());

                    //euclidean distance from this victim to the nearest refuge
                    target.setDistanceToRefuge(worldInfo.getDistance(human.getPosition(), findNearestRefuge(human.getPosition())));

                    target.setDistanceToPartition(getDistance.distance(victim.getLocation(worldInfo.getRawWorld()), clusterCenterMap.get(this.clusterIndex)));
                }
                //euclidean distance from this victim to the me
                target.setDistanceToMe(computingDistance(human));

                target.setVictimSituation(calculateVictimProfitability(human));

                targetsMap.put(victim.getID(), target);
            }
        }

    }
    /*private IntProperty BEDCAPACITY;
    private IntProperty OCCUPIEDBEDS;
    private IntProperty REFILLCAPACITY;
    private IntProperty WAITINGLISTSIZE;
    * */

    //版本正在写 todo:等待时间的计算还未实现
    /**
     * @author :Yhy
     * @description: 寻找床位
     * @Date:2021/4/13
     * */
    public EntityID findNearestRefuge(EntityID positionId) {
        double leastTime;      //花费最少时间
        double movingTime;     //移动到当前庇护所所需要的时间
        double movingNextTime; //从当前庇护所移动到下一庇护所所需要的时间
        double waitingTime = 0; //等待当前庇护所空出病床的时间
        Collection<StandardEntity> refuges = worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE);
        //将庇护所按照到达距离排好顺序
        StandardEntity[] result = sortedByDistance(refuges, positionId);

        EntityID nearestID = null;
        int nearestDistance = Integer.MAX_VALUE;
        int tempDistance;
        if (positionId != null && refuges != null && !refuges.isEmpty()) {
            //已经将医院排好顺序了,直接判断床位信息
            for (int i = 0; i < result.length - 2; i++){
                //医院床位足够
                Refuge refuge = new Refuge(result[i].getID());
                if((refuge.getBedCapacity() - refuge.getOccupiedBeds() >= 1)){
                    refuge.increaseOccupiedBeds();
                    return result[i].getID();
                } else {
                    //todo:当前医院床位不够,判断是不是要去下一个医院
                    movingTime = worldInfo.getDistance(positionId, result[i].getID()) / AGENT_CAN_MOVE; //当前医院到下一医院的时间
                    leastTime = waitingTime + movingTime;
                    //下一医院有床位
                    Refuge refuge1 = new Refuge(result[i + 1].getID());
                    movingNextTime = worldInfo.getDistance(positionId, result[i + 1].getID()) / AGENT_CAN_MOVE;
                    //到下一医院的时间要大于到当前医院的时间 + 当前等待时间  todo:等待当前医院空出病床
                    if(movingNextTime > leastTime){
                        refuge.setWaitingListSize((refuge.getWaitingListSize()) + 1);
                        return result[i].getID();
                    } else {//到下一个医院的时间小于当前医院所需总时间
                        if((refuge1.getBedCapacity() - refuge1.getOccupiedBeds()) >= 1){//可以直接入住
                            refuge1.increaseOccupiedBeds();
                            return refuge1.getID();
                        } else {//不可以直接入住
                            double temp = movingNextTime + waitingTime;
                            if(leastTime > temp){
                                leastTime = temp;
                                nearestID = refuge1.getID();
                            } else {
                                return refuge1.getID();
                            }
                        }
                    }
                }
            }
        }
        return nearestID;
    }
    /**
     * @author :Yhy
     * @description: 因为庇护所的床位数有限,所以不能直接寻找最佳庇护所
     * @param : refuges:所有的庇护所  positionID:位置
     * */

    //todo:将到庇护所的距离进行排序,因为床位数有限所以要进行判断
    private StandardEntity[] sortedByDistance(Collection<StandardEntity> refuges, EntityID positionID){
        int tempDistance, nearestDistance;
        StandardEntity[] res = (StandardEntity[]) refuges.toArray(new StandardEntity[refuges.size()]);
        for (int i = 0; i < res.length - 1; i++){
            for (int j = 0; j < res.length - i - 1; j++){
                tempDistance = worldInfo.getDistance(res[j].getID(), positionID);
                nearestDistance = worldInfo.getDistance(res[j + 1].getID(), positionID);
                if(tempDistance > nearestDistance){
                    StandardEntity temp = res[j];
                    res[j] = res[j + 1];
                    res[j + 1] = temp;
                }
            }
        }
        return res;
    }


    /**
     * This method computes distance based on euclidean distance.
     * <br>
     * <br>
     * <b>Note:</b> Based on human importance, the distance may be changed to lower value
     *
     * @param human the human to calculate its distance to me.
     * @return euclidean distance from human to me
     */
    private int computingDistance(Human human) {

        double coefficient = 1;
        if (human instanceof AmbulanceTeam) {
            coefficient = 0.90;
        } else if (human instanceof PoliceForce) {
            coefficient = 0.95;
        } else if (human instanceof FireBrigade) {
            coefficient = 0.97;
        } else {//human is instance of Civilian
            coefficient = 1;
        }

        return (int) (coefficient * getDistance.distance(human.getPosition(worldInfo.getRawWorld()).getLocation(worldInfo.getRawWorld()), worldInfo.getLocation(agentInfo.me())));
    }

//    private void possibleRefuges(Collection<StandardEntity> refuges, EntityID positionID){
//        int distance1;
//        int distance2;
//        int tempDistance;
//        int nearestDistance = Integer.MAX_VALUE;
//        EntityID nearestID = null;
//        StandardEntity tempEntity;
//        StandardEntity bestEntity;
//
//        Iterator it =  refuges.iterator();
//        while (it.hasNext()){
//            tempEntity = (StandardEntity) it.next();
//            if(tempEntity.getCount > 0){
//                tempDistance = worldInfo.getDistance(tempEntity.getID(), positionID);
//                if(tempDistance < nearestDistance){
//                    nearestDistance = tempDistance;
//                    nearestID =
//                }
//            }
//        }
//    }

    /**
     * calculates victim profitability
     *
     * @param human target human (kossher)
     * @return
     */
    private double calculateVictimProfitability(Human human) {

        int ttd = (int) Math.ceil(human.getHP() / (double) human.getDamage() * 0.8); //a pessimistic time to death

        double profitability = 100 / (double) ((human.getBuriedness() * ttd) + 1);

        return profitability;
    }

    public static class getDistance{
        public static int distance(Area obj1, Area obj2) {
            return distance(obj1.getX(), obj1.getY(), obj2.getX(), obj2.getY());
        }

        public static int distance(Pair<Integer, Integer> obj1, Pair<Integer, Integer> obj2) {
            return distance(obj1.first(), obj1.second(), obj2.first(), obj2.second());
        }
        public static int distance(int x1, int y1, int x2, int y2) {
            float dx = x1 - x2;
            float dy = y1 - y2;
            return (int) Math.sqrt(dx * dx + dy * dy);
        }

        public static double distance(double x1, double y1, double x2, double y2) {
            double dx = x1 - x2;
            double dy = y1 - y2;
            return (double) Math.sqrt(dx * dx + dy * dy);
        }

        public static int distance(Point p1, Point p2) {
            double dx = p1.getX() - p2.getX();
            double dy = p1.getY() - p2.getY();
            return (int) Math.sqrt(dx * dx + dy * dy);
        }

        public static int distance(Point2D p1, Point2D p2) {
            double dx = p1.getX() - p2.getX();
            double dy = p1.getY() - p2.getY();
            return (int) Math.sqrt(dx * dx + dy * dy);
        }

        public static int distance(rescuecore2.misc.geometry.Point2D start, rescuecore2.misc.geometry.Point2D end) {
            double dx = start.getX() - end.getX();
            double dy = start.getY() - end.getY();
            return (int) Math.sqrt(dx * dx + dy * dy);
        }

        public static int distance(Point point, Pair<Integer, Integer> pair) {
            double dx = point.getX() - pair.first();
            double dy = point.getY() - pair.second();
            return (int) Math.sqrt(dx * dx + dy * dy);
        }

        public static int distance(Pair<Integer, Integer> pair, Point point) {
            double dx = point.getX() - pair.first();
            double dy = point.getY() - pair.second();
            return (int) Math.sqrt(dx * dx + dy * dy);
        }

    }
}
