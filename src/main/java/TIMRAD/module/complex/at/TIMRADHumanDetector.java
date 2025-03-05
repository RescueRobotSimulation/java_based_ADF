package TIMRAD.module.complex.at;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.communication.standard.bundle.MessageUtil;
import adf.core.agent.communication.standard.bundle.information.MessageBuilding;
import adf.core.agent.communication.standard.bundle.information.MessageCivilian;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.agent.precompute.PrecomputeData;
import adf.core.component.communication.CommunicationMessage;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.HumanDetector;
import adf.core.launcher.ConsoleOutput;
import rescuecore2.standard.entities.*;
import rescuecore2.worldmodel.EntityID;

import java.io.Serializable;
import java.util.*;
import java.util.stream.Collectors;


/**
 * @author Shiva
 */

public class TIMRADHumanDetector extends HumanDetector{
    private EntityID result;
    private int clusterIndex;
    private DistanceBasedTargetSelector targetSelector;
    private VictimClassifier victimClassifier;
    private Clustering clustering;
    private Map<EntityID, BuildingProperty> sentBuildingMap;
    private Map<EntityID, Integer> sentTimeMap;
    private PathPlanning pathPlanning;
    private Map<StandardEntity, Integer> blockedVictims;
    private Random rnd = new Random(System.currentTimeMillis());
    private static final int DAMAGE_THRESHOLD = 50; // Threshold for considering a victim non-recoverable
    private static final int MIN_AMBULANCE_SUPPORT = 2; // Minimum number of ambulances required to save a high-damage victim

    public TIMRADHumanDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);

        switch (scenarioInfo.getMode()) {
            case PRECOMPUTATION_PHASE:
            case PRECOMPUTED:
            case NON_PRECOMPUTE:
                this.pathPlanning = moduleManager.getModule("SampleRoadDetector.PathPlanning", "adf.impl.module.algorithm.AStarPathPlanning");
                this.clustering = moduleManager.getModule("SampleRoadDetector.Clustering", "adf.impl.module.algorithm.SampleKMeans");
                break;
        }

        this.blockedVictims = new HashMap<>();
        this.clusterIndex = -1;
        this.sentBuildingMap = new HashMap<>();
        this.sentTimeMap = new HashMap<>();
        targetSelector = new DistanceBasedTargetSelector2(ai, wi, si, moduleManager, developData);
        initClassifier();
    }

    @Override
    public HumanDetector calc() {
        if (clustering == null) {
            this.result = this.failedClusteringCalc();
            return this;
        }

        if (this.clusterIndex == -1) {
            this.clusterIndex = clustering.getClusterIndex(this.agentInfo.getID());
        }

        Collection<StandardEntity> elements = clustering.getClusterEntities(this.clusterIndex);
        updateBlockedVictims();

        victimClassifier.updateGoodHumanList(worldInfo.getEntitiesOfType(StandardEntityURN.CIVILIAN));
        prioritizeVictimsBasedOnCondition();


        for (int i = 0; i < 6; i++) {
            victimClassifier.getMyGoodHumans().removeAll(blockedVictims.keySet());
            result = selectTarget(victimClassifier.getMyGoodHumans());

            if (result != null) {
                StandardEntity position = worldInfo.getPosition(result);
                if (position != null) {
                    List<EntityID> path = pathPlanning.getResult(agentInfo.getPosition(), position.getID());
                    if (agentInfo.getPosition().equals(position.getID()) || path != null && !path.isEmpty()) {
                        return this;
                    }
                    int postponeTime = rnd.nextInt(6) + 5;
                    blockedVictims.put(worldInfo.getEntity(result), postponeTime);
                }
            }
        }

        return this;
    }

    // Update blocked victims and their postponed time
    private void updateBlockedVictims() {
        ArrayList<StandardEntity> toRemove = new ArrayList<>();
        int postponeTime;
        for (StandardEntity standardEntity : blockedVictims.keySet()) {
            postponeTime = blockedVictims.get(standardEntity);
            postponeTime--;
            if (postponeTime <= 0) {
                toRemove.add(standardEntity);
            } else {
                blockedVictims.put(standardEntity, postponeTime);
            }
        }
        blockedVictims.keySet().removeAll(toRemove);
    }

    private void prioritizeVictimsBasedOnCondition() {
        // Sort victims based on severity of their condition (e.g., health status, damage level)
        victimClassifier.getMyGoodHumans().sort((v1, v2) -> {
            int priority1 = getPriorityForVictim(v1);
            int priority2 = getPriorityForVictim(v2);
            return Integer.compare(priority2, priority1);
        });
    }

    // Compute priority for a victim based on health, damage, and other factors
    private int getPriorityForVictim(Human victim) {
        int priority = 0;

        if (victim.isHPDefined() && victim.getHP() < 1000) {
            priority += 10;
        }

        if (victim.isDamageDefined() && victim.getDamage() > DAMAGE_THRESHOLD) {
            priority -= 100;
        } else {
            priority += 5;
        }

        if (victim.isPositionDefined()) {
            StandardEntity position = worldInfo.getEntity(victim.getPosition());
            if (position instanceof Building && ((Building) position).isFierynessDefined() && ((Building) position).getFieryness() > 0) {
                priority += 8;
            }
        }

        return priority;
    }

    // Select the target based on various factors such as distance, priority, and availability
    private EntityID selectTarget(List<Human> victims) {
        List<Human> prioritizedVictims = new ArrayList<>(victims);
        prioritizedVictims.sort((v1, v2) -> {
            int distance1 = calculateDistanceToVictim(v1);
            int distance2 = calculateDistanceToVictim(v2);
            return Integer.compare(distance1, distance2);
        });

        for (Human victim : prioritizedVictims) {
            if (isVictimAccessible(victim) && isSupportSufficient(victim)) {
                return victim.getID();
            }
        }

        return null;
    }

    // Calculate the distance to a victim (this could be based on the victim's position and agent's position)
    private int calculateDistanceToVictim(Human victim) {
        StandardEntity victimPosition = worldInfo.getPosition(victim.getID());
        if (victimPosition != null) {
            return agentInfo.getPosition().distanceTo(victimPosition.getID());
        }
        return Integer.MAX_VALUE;
    }

    // Check if a victim is accessible based on environmental factors (e.g., obstacles, buildings)
    private boolean isVictimAccessible(Human victim) {
        StandardEntity victimPosition = worldInfo.getPosition(victim.getID());
        return victimPosition != null && worldInfo.getEntity(victimPosition.getID()) instanceof Building;
    }


    private boolean isSupportSufficient(Human victim) {
        if (victim.isDamageDefined() && victim.getDamage() > DAMAGE_THRESHOLD) {
            int nearbyAmbulances = countNearbyAmbulances(victim);
            return nearbyAmbulances >= MIN_AMBULANCE_SUPPORT;
        }
        return true;
    }

    // Count the number of ambulances near a victim
    private int countNearbyAmbulances(Human victim) {
        int count = 0;
        for (StandardEntity entity : worldInfo.getEntitiesOfType(StandardEntityURN.AMBULANCE_TEAM)) {
            if (entity instanceof AmbulanceTeam) {
                AmbulanceTeam ambulance = (AmbulanceTeam) entity;
                if (calculateDistanceToVictim(victim) <= ambulance.getPosition().distanceTo(victim.getPosition())) {
                    count++;
                }
            }
        }
        return count;
    }


    private EntityID failedClusteringCalc() {
        List<Human> targets = new ArrayList<>();
        for (StandardEntity next : worldInfo.getEntitiesOfType(StandardEntityURN.CIVILIAN, StandardEntityURN.FIRE_BRIGADE, StandardEntityURN.POLICE_FORCE, StandardEntityURN.AMBULANCE_TEAM)) {
            Human h = (Human) next;
            if (agentInfo.getID() == h.getID()) {
                continue;
            }
            if (h.isHPDefined() && h.isBuriednessDefined() && h.isDamageDefined() && h.isPositionDefined() && h.getHP() > 0) {
                targets.add(h);
            }
        }
        targets.sort(new DistanceSorter(this.worldInfo, this.agentInfo.getPositionArea()));
        return targets.isEmpty() ? null : targets.get(0).getID();
    }

    @Override
    public EntityID getTarget() {
        return this.result;
    }

    private void initClassifier() {
        victimClassifier = new VictimClassifier2(worldInfo, agentInfo);
    }

    @Override
    public HumanDetector precompute(PrecomputeData precomputeData) {
        super.precompute(precomputeData);
        this.clustering.precompute(precomputeData);
        return this;
    }

    @Override
    public HumanDetector resume(PrecomputeData precomputeData) {
        super.resume(precomputeData);
        this.clustering.resume(precomputeData);
        return this;
    }

    @Override
    public HumanDetector preparate() {
        super.preparate();
        this.clustering.preparate();
        return this;
    }

    @Override
    public HumanDetector updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);

        if (this.getCountUpdateInfo() >= 2) {
            return this;
        }

        this.clustering.updateInfo(messageManager);

        Set<EntityID> inBuildingAmbulances = new HashSet<>();
        for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
            StandardEntity entity = worldInfo.getEntity(id);
            if (entity instanceof AmbulanceTeam) {
                StandardEntity position = worldInfo.getPosition(entity.getID());
                if (position instanceof Building && !entity.getID().equals(agentInfo.getID())
                        && position.getID().equals(agentInfo.getPosition())) {
                    inBuildingAmbulances.add(entity.getID());
                }
            }
        }

        for (EntityID id : worldInfo.getChanged().getChangedEntities()) {
            StandardEntity entity = worldInfo.getEntity(id);
            if (entity instanceof Building) {
                Building building = (Building) entity;
                if (building.isFierynessDefined() && building.getFieryness() > 0) {
                    sentBuildingMap.compute(id, (key, oldProperty) -> {
                        if (oldProperty == null || oldProperty.getFieryness() != building.getFieryness()) {
                            messageManager.addMessage(new MessageBuilding(true, building));
                            messageManager.addMessage(new MessageBuilding(false, building));
                            return new BuildingProperty(building);
                        }
                        return oldProperty;
                    });
                }
            } else if (entity instanceof Civilian) {
                Civilian civilian = (Civilian) entity;
                boolean isCritical = civilian.isHPDefined() && civilian.getHP() > 1000
                        && civilian.isDamageDefined() && civilian.getDamage() > 0;
                boolean isInBuilding = civilian.isPositionDefined()
                        && !(worldInfo.getEntity(civilian.getPosition()) instanceof Refuge)
                        && (worldInfo.getEntity(civilian.getPosition()) instanceof Building);

                if ((isCritical || isInBuilding) && inBuildingAmbulances.size() < 3) {
                    messageManager.addMessage(new MessageCivilian(true, civilian));
                    messageManager.addMessage(new MessageCivilian(false, civilian));
                }
            }
        }

        this.reflectMessage(messageManager);
        return this;
    }


    private void reflectMessage(MessageManager messageManager) {
        Set<EntityID> changedEntities = new HashSet<>(this.worldInfo.getChanged().getChangedEntities());
        changedEntities.add(this.agentInfo.getID());

        messageManager.getReceivedMessageList().stream()
                .filter(message -> message instanceof MessageCivilian)
                .map(message -> (MessageCivilian) message)
                .forEach(mc -> MessageUtil.reflectMessage(this.worldInfo, mc));
    }


}
