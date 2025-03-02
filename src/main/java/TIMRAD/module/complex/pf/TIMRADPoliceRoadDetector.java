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

        openedAreas.add((Area) currentPosition);

        // اگر به هدف رسیدیم، آن را ریست کنیم
        if (positionID.equals(this.target)) {
            this.target = null;
        }

        // اگر هدفی نداریم، هدف جدید پیدا کنیم
        if (this.target == null) {
            Set<Area> targets = findTargetAreas();
              System.out.println("Road Detector currentTargets :" + targets);


            if (targets.isEmpty()) {
                this.target = null;
                return this;
            }

            // پیدا کردن نزدیک‌ترین هدف با PathPlanning
            this.pathPlanning.setFrom(positionID);
            this.pathPlanning.setDestination(targets.stream().map(Area::getID).collect(Collectors.toSet()));
            List<EntityID> path = this.pathPlanning.calc().getResult();

            if (path != null && !path.isEmpty()) {
                this.target = path.get(path.size() - 1); // آخرین نقطه مسیر = هدف
            } else {
                this.target = null;
            }
        }

        return this;
    }

// متد کمکی برای پیدا کردن مناطق هدف
private Set<Area> findTargetAreas() {
    Set<Area> targetAreas = new HashSet<>();

    // 1. پناهگاه‌ها
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.REFUGE)) {
        targetAreas.add((Area) entity);
        logger.debug("Added refuge: " + entity.getID());
    }

    // 2. انسان‌های گرفتار و عامل‌های گرفتار
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(
            StandardEntityURN.CIVILIAN,
            StandardEntityURN.AMBULANCE_TEAM,
            StandardEntityURN.FIRE_BRIGADE,
            StandardEntityURN.POLICE_FORCE)) {
        if (isValidHuman(entity)) {
            Human human = (Human) entity;
            Area humanPosition = (Area) worldInfo.getEntity(human.getPosition());
            targetAreas.add(humanPosition);
            logger.debug("Added trapped human/agent position: " + humanPosition.getID());
        }
    }

    // 3. ورودی‌های مسدود ساختمان‌ها
    for (StandardEntity entity : this.worldInfo.getEntitiesOfType(StandardEntityURN.BUILDING, StandardEntityURN.REFUGE, StandardEntityURN.GAS_STATION)) {
        Building building = (Building) entity;
        for (EntityID entranceID : building.getNeighbours()) {
            StandardEntity entrance = worldInfo.getEntity(entranceID);
            if (entrance instanceof Road) {
                Road road = (Road) entrance;
                if (road.isBlockadesDefined() && !road.getBlockades().isEmpty()) {
                    targetAreas.add(road);
                    logger.debug("Added blocked entrance road: " + road.getID());
                }
            }
        }
    }

    // حذف مناطق باز شده
    targetAreas.removeAll(openedAreas);
    return targetAreas;
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