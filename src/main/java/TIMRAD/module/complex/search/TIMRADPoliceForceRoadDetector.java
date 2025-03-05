package TIMRAD.module.complex.pf;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.centralized.CommandPolice;
import adf.core.agent.communication.standard.bundle.information.*;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.communication.CommunicationMessage;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.RoadDetector;
import adf.core.launcher.ConsoleOutput;
import com.mrl.debugger.remote.VDClient;
import mrl_2023.algorithm.clustering.ConvexHull;
import mrl_2023.complex.firebrigade.BuildingProperty;
import mrl_2023.util.Util;
import mrl_2023.viewer.MrlPersonalData;
import mrl_2023.world.MrlWorldHelper;
import mrl_2023.world.entity.Entrance;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.awt.*;
import java.io.Serializable;
import java.util.List;
import java.util.*;

import static rescuecore2.standard.entities.StandardEntityURN.*;


/**
 * @author Shiva
 */

public class TIMRADPoliceForceRoadDetector extends RoadDetector{

    private Set<EntityID> targetAreas, doneTasks, priorityRoads, coincidentalRoads;
    private Map<StandardEntityURN, Set<EntityID>> targetAreasMap;
    private PathPlanning pathPlanning;
    private Clustering clustering;
    private EntityID result;
    private Polygon clusterConvexPolygon;
    private Map<EntityID, BuildingProperty> sentBuildingMap;
    private MrlWorldHelper worldHelper;


    public TIMRADPoliceRoadDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning", "adf.impl.module.algorithm.AStarPathPlanning");
        this.clustering = moduleManager.getModule("MrlSimpleFireSearch.Clustering.Police", "adf.impl.module.algorithm.KMeansClustering");
        this.worldHelper = WorldHelper.load(agentInfo, worldInfo, scenarioInfo, moduleManager, developData); //WorldHelper --> TIMRADWorldHelper

        this.result = null;
        this.sentBuildingMap = new HashMap<>();
        this.coincidentalRoads = new HashSet<>();
        this.priorityRoads = new HashSet<>();
        this.targetAreas = new HashSet<>();
        this.doneTasks = new HashSet<>();
        this.targetAreasMap = new HashMap<>();
    }

    @Override
    public RoadDetector calc() {
        EntityID positionID = this.agentInfo.getPosition();
        updateBlockedEntities();

        coincidentalRoads.removeAll(doneTasks);
        if (result != null && coincidentalRoads.contains(result)) {
            return this;
        }
        if (!coincidentalRoads.isEmpty()) {
            return getRoadDetector(positionID, coincidentalRoads);
        }

        filterTargetAreas();
        if (priorityRoads.contains(positionID) || targetAreas.contains(positionID)) {
            result = positionID;
            return this;
        }

        return selectBestTarget(positionID);
    }
    private void updateBlockedEntities() {
        for (StandardEntity entity : worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE, StandardEntityURN.AMBULANCE_TEAM, StandardEntityURN.REFUGE)) {
            if (worldInfo.getChanged().getChangedEntities().contains(entity.getID())) {
                Building building = entity instanceof Building ? (Building) entity : null;
                if (building != null) addBuildingNeighboursToTargets(building);
            }
        }
    }

    private void addBuildingNeighboursToTargets(Building building) {
        for (EntityID neighbourId : building.getNeighbours()) {
            StandardEntity neighbour = worldInfo.getEntity(neighbourId);
            if (neighbour instanceof Road && !doneTasks.contains(neighbour.getID())) {
                coincidentalRoads.add(neighbour.getID());
            }
        }
    }

    private void filterTargetAreas() {
        List<EntityID> removeList = new ArrayList<>();
        for (EntityID id : priorityRoads) {
            if (!targetAreas.contains(id)) removeList.add(id);
        }
        priorityRoads.removeAll(removeList);
        priorityRoads.removeAll(doneTasks);
        targetAreasMap.values().forEach(set -> set.removeAll(removeList));
    }

    private RoadDetector selectBestTarget(EntityID positionID) {
        for (StandardEntityURN type : List.of(StandardEntityURN.FIRE_BRIGADE, StandardEntityURN.REFUGE, StandardEntityURN.AMBULANCE_TEAM)) {
            Set<EntityID> entityIDSet = targetAreasMap.get(type);
            if (entityIDSet != null && !entityIDSet.isEmpty()) return getRoadDetector(positionID, entityIDSet);
        }
        if (!priorityRoads.isEmpty()) return getRoadDetector(positionID, priorityRoads);

        pathPlanning.setFrom(positionID);
        pathPlanning.setDestination(targetAreas);
        List<EntityID> path = pathPlanning.calc().getResult();
        if (path != null && !path.isEmpty()) result = path.get(path.size() - 1);
        return this;
    }

    private RoadDetector getRoadDetector(EntityID positionID, Set<EntityID> entityIDSet) {
        pathPlanning.setFrom(positionID);
        pathPlanning.setDestination(entityIDSet);
        List<EntityID> path = pathPlanning.calc().getResult();
        if (path != null && !path.isEmpty()) result = path.get(path.size() - 1);
        return this;
    }

    @Override
    public EntityID getTarget() {
        return result;
    }




    @Override
    public RoadDetector precompute(PrecomputeData precomputeData) {
    super.precompute(precomputeData);
    if (this.getCountPrecompute() >= 2) {
        return this;
    }
    this.pathPlanning.precompute(precomputeData);
    return this;
    }

    @Override
    public RoadDetector resume(PrecomputeData precomputeData) {
    super.resume(precomputeData);
    if (this.getCountResume() >= 2) {
        return this;
    }
    this.pathPlanning.resume(precomputeData);
    fillTargets();
    return this;
    }


    private void fillTargets(){

    int priority = road.getNeighbours().size();

    for (StandardEntity entity : worldInfo.getEntitiesOfType(StandardEntityURN.FIRE_BRIGADE, StandardEntityURN.AMBULANCE_TEAM)) {
        if (worldInfo.getPosition(entity.getID()).equals(road)) {
            priority += 5;
        }
    }

    if (isBlocked(road)) {
        double blockadeRatio = (double) road.getBlockades().size() / road.getNeighbours().size();
        priority -= (int) (blockadeRatio * 5); // کاهش بر اساس شدت انسداد

        // افزایش اولویت در صورتی که عامل در جاده مسدود گیر افتاده باشد
        if (agentInfo.getPosition().equals(road.getID())) {
            priority += 10; 
        }
    }

    // افزایش اولویت برای جاده‌هایی که به پناهگاه متصل هستند
    for (EntityID neighborID : road.getNeighbours()) {
        StandardEntity neighbor = worldInfo.getEntity(neighborID);
        if (neighbor instanceof Refuge) {
            priority += 5; 
        }


    }

    @Override
    public RoadDetector preparate() {
        super.preparate();
        if (this.getCountPreparate() >= 2) {
            return this;
        }
        this.pathPlanning.preparate();
        fillTargets();
        return this;
    }

    @Override
    public RoadDetector updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);


        if (MrlPersonalData.DEBUG_MODE) {
            ArrayList<Polygon> data = new ArrayList<>();
            data.add(this.clusterConvexPolygon);
            VDClient.getInstance().drawAsync(agentInfo.getID().getValue(), "ClusterConvexPolygon", data);
        }

        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }
        this.pathPlanning.updateInfo(messageManager);
        if (this.result != null) {
            if (worldInfo.getChanged().getChangedEntities().contains(this.result)) {
                StandardEntity entity = this.worldInfo.getEntity(this.result);
                if (entity instanceof Building) {
                    this.result = null;
                } else if (entity instanceof Road) {
                    Road road = (Road) entity;
                    if ((!road.isBlockadesDefined() || road.getBlockades().isEmpty()) && agentInfo.getPosition().equals(road.getID())) {
                        this.doneTasks.add(this.result);
                        this.targetAreas.remove(this.result);
                        this.result = null;
                    }
                }
            }
        }

        worldInfo.getChanged().getChangedEntities().forEach(changedId -> {
            StandardEntity entity = worldInfo.getEntity(changedId);
            if (entity instanceof Building) {
                Building building = (Building) worldInfo.getEntity(changedId);
                if (building.isFierynessDefined() && building.getFieryness() > 0 /*|| building.isTemperatureDefined() && building.getTemperature() > 0*/) {
                    BuildingProperty buildingProperty = sentBuildingMap.get(changedId);
                    if (buildingProperty == null || buildingProperty.getFieryness() != building.getFieryness() || buildingProperty.getFieryness() == 1) {
                        messageManager.addMessage(new MessageBuilding(true, building));
                        messageManager.addMessage(new MessageBuilding(false, building));
                        sentBuildingMap.put(changedId, new BuildingProperty(building));
                    }
                }
            } else if (entity instanceof Civilian) {
                Civilian civilian = (Civilian) entity;
                if ((civilian.isHPDefined() && civilian.getHP() > 1000 && civilian.isDamageDefined() && civilian.getDamage() > 0)
                        || ((civilian.isPositionDefined() && !(worldInfo.getEntity(civilian.getPosition()) instanceof Refuge))
                        && (worldInfo.getEntity(civilian.getPosition()) instanceof Building))) {
                    messageManager.addMessage(new MessageCivilian(true, civilian));
                    messageManager.addMessage(new MessageCivilian(false, civilian));
                    StandardEntity target = this.worldInfo.getPosition(civilian.getID());
                    if (target instanceof Building) {
                        if (isInMyTerritoryOrCloseToIt(target)) {
                            for (EntityID id : ((Building) target).getNeighbours()) {
                                StandardEntity neighbour = this.worldInfo.getEntity(id);
                                if (neighbour instanceof Road) {
                                    this.priorityRoads.add(id);
                                    this.targetAreas.add(id);
                                }
                            }
                            List<Entrance> entrances = worldHelper.getMrlBuilding(target.getID()).getEntrances();
                            if (entrances != null && !entrances.isEmpty()) {
                                entrances.forEach(entrance -> {
                                    this.priorityRoads.add(entrance.getID());
                                    this.targetAreas.add(entrance.getID());
                                });
                            }
                        }
                    }
                }

            }
        });


        Set<EntityID> changedEntities = this.worldInfo.getChanged().getChangedEntities();
        for (CommunicationMessage message : messageManager.getReceivedMessageList()) {
            Class<? extends CommunicationMessage> messageClass = message.getClass();
            if (messageClass == MessageAmbulanceTeam.class) {
                this.reflectMessage((MessageAmbulanceTeam) message);
            } else if (messageClass == MessageFireBrigade.class) {
                this.reflectMessage((MessageFireBrigade) message);
            } else if (messageClass == MessageRoad.class) {
                this.reflectMessage((MessageRoad) message, changedEntities);
            } else if (messageClass == MessagePoliceForce.class) {
                this.reflectMessage((MessagePoliceForce) message);
            } else if (messageClass == CommandPolice.class) {
                this.reflectMessage((CommandPolice) message);
            } else if (messageClass == MessageCivilian.class) {
                MessageCivilian mc = (MessageCivilian) message;
                if (!changedEntities.contains(mc.getAgentID())) {
                    MessageUtil.reflectMessage(this.worldInfo, mc);
                    StandardEntity target = this.worldInfo.getPosition(mc.getAgentID());
                    if (isInMyTerritoryOrCloseToIt(target)) {
                        if (target instanceof Building) {
                            for (EntityID id : ((Building) target).getNeighbours()) {
                                StandardEntity neighbour = this.worldInfo.getEntity(id);
                                if (neighbour instanceof Road) {
agentInfo.getID() + " civId: " + mc.getAgentID() + " roadId: " + neighbour.getID());
                                    this.priorityRoads.add(id);
                                    this.targetAreas.add(id);
                                }
                            }
                            List<Entrance> entrances = worldHelper.getMrlBuilding(target.getID()).getEntrances();
                            if (entrances != null && !entrances.isEmpty()) {
                                entrances.forEach(entrance -> {
                                    this.priorityRoads.add(entrance.getID());
                                    this.targetAreas.add(entrance.getID());
                                });
                            }

                        }
                    }


                }
            }
        }
        for (EntityID id : this.worldInfo.getChanged().getChangedEntities()) {
            StandardEntity entity = this.worldInfo.getEntity(id);
            if (entity instanceof Road) {
                Road road = (Road) entity;
                if (!road.isBlockadesDefined() || road.getBlockades().isEmpty()) {
                    this.doneTasks.add(id);
                    this.targetAreas.remove(id);
                }
            }
        }
        return this;
    }


}

}
