package TIMRAD.module.complex.pf;

import adf.core.agent.communication.MessageManager;
import adf.core.agent.develop.DevelopData;
import adf.core.agent.info.AgentInfo;
import adf.core.agent.info.ScenarioInfo;
import adf.core.agent.info.WorldInfo;
import adf.core.agent.module.ModuleManager;
import adf.core.component.module.algorithm.Clustering;
import adf.core.component.module.algorithm.PathPlanning;
import adf.core.component.module.complex.RoadDetector;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.log4j.Logger;

import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.EntityID;

public class TIMRADPoliceRoadDetector extends RoadDetector {

    private PathPlanning pathPlanning;
    private Clustering clustering;
    private EntityID target;  // هدف انتخاب‌شده
    private Logger logger;    // برای لاگ‌گذاری

    private Set<Area> openedAreas = new HashSet<>();


    public TIMRADPoliceRoadDetector(AgentInfo ai, WorldInfo wi, ScenarioInfo si, ModuleManager moduleManager, DevelopData developData) {
        super(ai, wi, si, moduleManager, developData);
        
        // تنظیم لاگ‌گذاری
        this.logger = Logger.getLogger(TIMRADPoliceRoadDetector.class);

        // بارگذاری ماژول‌ها
        this.pathPlanning = moduleManager.getModule(
            "TIMRADPoliceRoadDetector.PathPlanning",
            "adf.impl.module.algorithm.DijkstraPathPlanning"
        );
        this.clustering = moduleManager.getModule(
            "TIMRADPoliceRoadDetector.Clustering",
            "adf.impl.module.algorithm.KMeansClustering"
        );

        // ثبت ماژول‌ها
        registerModule(this.pathPlanning);
        registerModule(this.clustering);

        // مقداردهی اولیه هدف
        this.target = null;
    }

    @Override
    public RoadDetector updateInfo(MessageManager messageManager) {
        super.updateInfo(messageManager);
        return this;
    }


  @Override
  public RoadDetector calc() {
    EntityID positionID = this.agentInfo.getPosition();
    StandardEntity currentPosition = worldInfo.getEntity(positionID);
    logger.debug("Current position: " + currentPosition);

    // اضافه کردن موقعیت فعلی به مناطق باز شده
    openedAreas.add((Area) currentPosition);

    // اگر به هدف رسیدیم، آن را ریست کنیم
    if (positionID.equals(this.target)) {
        logger.debug("Reached target " + currentPosition + ", resetting target");
        this.target = null;
    }

    // اگر هدفی نداریم، هدف جدید پیدا کنیم
    if (this.target == null) {
        List<Set<Area>> prioritizedTargets = findPrioritizedTargetAreas();
        logger.debug("Prioritized targets: " + prioritizedTargets);

        // انتخاب هدف بر اساس اولویت
        for (Set<Area> targetSet : prioritizedTargets) {
            if (!targetSet.isEmpty()) {
                this.pathPlanning.setFrom(positionID);
                this.pathPlanning.setDestination(targetSet.stream().map(Area::getID).collect(Collectors.toSet()));
                List<EntityID> path = this.pathPlanning.calc().getResult();
                if (path != null && !path.isEmpty()) {
                    this.target = path.get(path.size() - 1); // نزدیک‌ترین هدف از این اولویت
                    logger.debug("Selected target: " + this.target + " from priority group");
                    break; // اولین گروه غیرخالی انتخاب می‌شود
                }
            }
        }

        if (this.target == null) {
            logger.debug("No targets found in any priority group");
        }
    }

    return this;
  }

    // متد کمکی برای پیدا کردن مناطق هدف
  private List<Set<Area>> findPrioritizedTargetAreas() {
    List<Set<Area>> prioritizedTargets = new ArrayList<>(6); // 6 سطح اولویت
    for (int i = 0; i < 6; i++) {
        prioritizedTargets.add(new HashSet<>());
    }

    // اولویت 0: پناهگاه‌ها
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE)) {
        prioritizedTargets.get(0).add((Area) entity);
        logger.debug("Added refuge (priority 0): " + entity.getID());
    }

    // اولویت 1: انسان‌ها (غیرنظامیان)
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.CIVILIAN)) {
        if (isValidHuman(entity)) {
            Human human = (Human) entity;
            Area humanPosition = (Area) worldInfo.getEntity(human.getPosition());
            prioritizedTargets.get(1).add(humanPosition);
            logger.debug("Added trapped civilian position (priority 1): " + humanPosition.getID());
        }
    }

    // اولویت 2: عامل‌های گرفتار (آتش‌نشان، آمبولانس، پلیس)
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(
            StandardEntityURN.AMBULANCE_TEAM,
            StandardEntityURN.FIRE_BRIGADE,
            StandardEntityURN.POLICE_FORCE)) {
        if (isValidHuman(entity)) {
            Human human = (Human) entity;
            Area humanPosition = (Area) worldInfo.getEntity(human.getPosition());
            prioritizedTargets.get(2).add(humanPosition);
            logger.debug("Added trapped agent position (priority 2): " + humanPosition.getID());
        }
    }

    // اولویت 3: ساختمان‌ها
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.BUILDING)) {
        Building building = (Building) entity;
        prioritizedTargets.get(3).add(building);
        logger.debug("Added building (priority 3): " + building.getID());
    }

    // اولویت 4: جاده‌ها
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.ROAD)) {
        Road road = (Road) entity;
        prioritizedTargets.get(4).add(road);
        logger.debug("Added road (priority 4): " + road.getID());
    }

    // اولویت 5: جاده‌های دارای موانع (Blockades)
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.ROAD)) {
        Road road = (Road) entity;
        if (road.isBlockadesDefined() && !road.getBlockades().isEmpty()) {
            prioritizedTargets.get(5).add(road);
            logger.debug("Added road with blockades (priority 5): " + road.getID());
        }
    }

    // فیلتر کردن به خوشه مأمور و حذف مناطق باز شده
    for (Set<Area> targetSet : prioritizedTargets) {
        Set<Area> inClusterTargets = filterInCluster(targetSet);
        inClusterTargets.removeAll(openedAreas);
        targetSet.clear();
        targetSet.addAll(inClusterTargets);
    }

    return prioritizedTargets;
  }


  private Set<Area> filterInCluster(Set<Area> targetAreas) {  
    int clusterIndex = this.clustering.getClusterIndex(this.agentInfo.getID());
    Set<Area> clusterTargets = new HashSet<>();
    Collection<StandardEntity> inClusterEntities = this.clustering.getClusterEntities(clusterIndex);

    for (Area target : targetAreas) {
        if (inClusterEntities.contains(target)) {
            clusterTargets.add(target);
        }
    }

    return clusterTargets;
  }


// متد کمکی برای اعتبارسنجی انسان‌ها
private boolean isValidHuman(StandardEntity entity) {
    if (!(entity instanceof Human)) return false;
    Human human = (Human) entity;
    if (!human.isHPDefined() || human.getHP() <= 0) return false; // مرده نباشد
    if (!human.isPositionDefined()) return false; // موقعیت مشخص باشد
    if (!human.isDamageDefined() || human.getDamage() <= 0) return false; // آسیب دیده باشد
    if (!human.isBuriednessDefined() || human.getBuriedness() <= 0) return false; // مدفون باشد
    StandardEntity position = worldInfo.getPosition(human);
    return position != null && position.getStandardURN() != StandardEntityURN.REFUGE; // در پناهگاه نباشد
}

    @Override
    public EntityID getTarget() {
        return this.target;
    }
}